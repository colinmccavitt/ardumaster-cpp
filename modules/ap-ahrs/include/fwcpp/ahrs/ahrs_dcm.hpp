#pragma once

// Port of AP_AHRS_DCM's gyro-integration attitude core (slice 1), YAW
// drift correction (slice 2, commit 49960ca), and now the ROLL/PITCH half
// of drift correction (slice 3, this addition): everything in
// drift_correction(float deltat) EXCEPT the drift_correction_yaw() call at
// its top (that's slice 2). CPP-028. Upstream: AP_AHRS/AP_AHRS_DCM.h,
// AP_AHRS_DCM.cpp (Plane-4.7.0) - read directly from the pinned upstream
// worktree, not from training-data memory.
//
// SLICE 1 (matrix_update/normalize/check_matrix/renorm/reset/update, no
// drift correction at all - commit b0e2e6d) left omega_i_/omega_p_/
// omega_yaw_p_ as always-zero placeholder fields that matrix_update()
// already read every tick. SLICE 2 (commit 49960ca) started writing
// omega_yaw_p_ and omega_i_.z from real compass/GPS yaw fusion. SLICE 3
// (this addition) makes omega_p_ real and completes omega_i_.x/omega_i_.y.
// matrix_update() ITSELF HAS NOT CHANGED across any of the three slices -
// not one line - it already consumed these exact fields in slice 1.
//
// With slice 3 landed, this class has NO axis left that drifts
// unboundedly: roll/pitch are now pulled back toward the accel-vs-gravity
// (and, with GPS, accel-vs-GPS-velocity-implied-gravity) reference exactly
// as upstream does, on top of slice 2's yaw correction. Everything else
// slice 1's banner already excluded for the same reasons (airspeed sensor
// hardware, groundspeed/position tracking, status/arming plumbing/GCS,
// backup_attitude()/watchdog persistence) is still excluded - see the
// SLICE 3 section below for what specifically is and isn't reproduced from
// drift_correction()'s roll/pitch half.
//
// COMPASS_CAL_ENABLED's compass.is_calibrating() early return is cut too -
// no compass calibration subsystem in this port.
//
// Compass::calculate_heading() IS ported, as a private helper taking the
// already-known field vector and declination instead of a live Compass&
// (checked upstream first, per the task's own instruction: it's a
// genuinely self-contained handful of lines - tilt-compensated atan2 of
// the earth-frame horizontal field components computed directly from
// dcm_matrix.c, plus a declination correction - not a large subsystem, so
// porting the one call site's real math beat stubbing it behind a
// pre-computed-heading parameter).
//
// This class now reproduces the FULL upstream drift-correction algorithm
// on all three axes (modulo the named, documented exclusions below) -
// nothing left drifts unbounded under pure gyro integration.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching this port's
// standing L1Inputs/AcPid/FilterRegistry pattern:
//   - GyroSample REPLACES AP::ins(): upstream's matrix_update() reaches
//     into the global AP_InertialSensor for get_delta_angle()/get_gyro().
//     Here the caller (whoever owns the real IMU driver) reads those and
//     hands them in per tick as GyroSample's three fields, which are
//     exactly upstream's local `delta_angle`/`dangle_dt` pair plus
//     `_ins.get_gyro()` - nothing added, nothing renamed away from its
//     upstream meaning.
//   - reset()'s `initial_accel` parameter REPLACES upstream's
//     `_ins.get_accel()` + `while (initAccVec.length() < 9 || > 11) {
//     _ins.wait_for_sample(); _ins.update(); }` retry loop. That loop is a
//     real hardware-polling concern - "keep asking the IMU driver for
//     samples until one looks like a plausible ~1g reading" - which
//     belongs to whoever owns the actual sensor, not to this pure-math
//     class. The contract here is the precondition upstream's loop was
//     defending: `initial_accel` must already be a valid ~1g reading (or,
//     for the "align flat" fallback, the caller is free to pass any accel
//     with length() <= 5.0f to get roll=pitch=0, matching upstream's own
//     fallback threshold - see reset()'s body).
//   - The watchdog-persistent-data recovery branch
//     (`hal.util->was_watchdog_reset() && AP_HAL::millis() < 10000`) is cut
//     entirely, not replaced - there is no watchdog/persistent-data
//     subsystem in this port to recover from. reset()'s only two paths are
//     upstream's remaining two: recover-from-eulers, and recover-from-accel.
//
// SLICE 2 EXTENDS THE SAME PATTERN for drift_correction_yaw() and friends:
//   - CompassSample REPLACES Compass&: `field`/`declination_rad` are
//     get_field()/get_declination(); `last_update_usec` is
//     last_update_usec(); `healthy` collapses use_for_yaw() - this port has
//     no compass health/calibration state machine to model beyond one bool.
//   - GpsSample REPLACES AP_GPS&: `ground_speed_ms`/`ground_course_deg`/
//     `last_fix_time_ms` are ground_speed()/ground_course()/
//     last_fix_time_ms(); `has_fix` collapses `status() > NO_FIX` (no
//     multi-level fix-status enum in this port).
//   - `gps_use_enabled` (plain bool) REPLACES `AP_Enum<GPSUse>& _gps_use` -
//     same "AP_Param not wired in yet" precedent as kp_yaw below;
//     upstream's `_gps_use == GPSUse::Disable` becomes `!gps_use_enabled`
//     inside have_gps().
//   - `fly_forward`/`armed`/`wind_speed_ms`/`now_ms` REPLACE
//     `AP::ahrs().get_fly_forward()`, `hal.util->get_soft_armed()`,
//     `_wind.xy().length()`, and `AP_HAL::millis()` respectively - explicit
//     per-call parameters instead of singleton/member reads, the same
//     treatment L1Inputs already gave AP_AHRS&/AP_TECS*. `wind_speed_ms`:
//     this port has no wind-estimation subsystem (see above), so the
//     caller supplies whatever wind-speed estimate it has (0 if none).
//   - `kp_yaw` (plain float, defaulted constructor parameter, default
//     0.2f matching upstream's `AHRS_YAW_P` GSCALAR default) REPLACES
//     `AP_Float& _kp_yaw` - same AcPid::Gains-style "AP_Param not wired in
//     yet" treatment. Unlike AcPid this is a single float bolted onto an
//     already-default-constructible class (slice 1 callers all do
//     `AhrsDcm ahrs;`), so it's a defaulted constructor parameter rather
//     than a mandatory Gains struct. `_ki_yaw`, by contrast, is NOT a
//     parameter: upstream declares it `static constexpr float _ki_yaw =
//     0.01f` (never an AP_Float), so it stays a compile-time constant
//     here too (kKiYaw).
//   - accel_ef (upstream: `_accel_ef`) is a plain public field, not
//     computed: it's normally populated by drift_correction()'s
//     accel-vs-gravity fusion, out of scope this slice (see above). Left
//     at its zero default, yaw_gain() always returns its maximum
//     (0.2*4.5 = 0.9) - the honest behavior of an unset field, not a bug.
//     A caller with a real earth-frame accel estimate may set it directly.
//
// Vector3::xy() ADDED (ap-math/vector3.hpp) to support this slice's
// yaw_gain() (`accel_ef.xy().length()`) - vector3.hpp's own banner had
// deferred xy() "until there's a concrete caller to design the safe
// equivalent against, rather than guessing at one speculatively." This is
// that caller. Implemented as `Vector2<T>(x, y)` (a copy), not upstream's
// reinterpret_cast aliasing - same ADR-0012 stance vector3.hpp's banner
// already cites for other functions.
//
// upstream's `compass.read()` FORCED RE-READ IS DROPPED:
// drift_correction_yaw()'s have_initial_yaw transition upstream calls
// `compass.read()` specifically "to throw away the first compass value,
// which can be bad." That is a live re-read of a stateful singleton
// driver - meaningless against this port's CompassSample, which is
// already the caller's current-tick snapshot with no second, fresher
// reading behind it to re-poll for. The transition here fires directly
// off that snapshot's field on the first tick compass_last_update_usec_
// genuinely advances. A caller wanting upstream's exact "discard one
// sample" behavior can hold back its own first CompassSample before
// calling in.
//
// _omega_I_sum BATCHING - SLICE 2's ORIGINAL NOTE (kept for history): a
// genuine divergence, not an oversight: upstream accumulates yaw's
// contribution into a SHARED `_omega_I_sum.z` (also written by the then-
// out-of-scope accel-based drift_correction(), same vector), only actually
// folded into `_omega_I` every 5 seconds (`_omega_I_sum_time >= 5`),
// constrained by `AP::ins().get_gyro_drift_rate()`. Since that
// batching/clamping machinery couldn't be ported without the accel half,
// slice 2 instead accumulated the yaw contribution directly and
// continuously into omega_i_.z on every gated tick - the identical
// `error_z * kKiYaw * yaw_deltat` term upstream computes, just applied
// immediately rather than batched-then-clamped.
//
// SLICE 3 UPDATE - PARTIAL UNIFICATION, not full: now that this slice adds
// the accel half's real x/y/z error contributions, the task of porting
// this class asked whether to now properly unify _omega_I_sum across both
// paths (upstream's actual behavior) or keep them separate. Full
// unification is NOT practical without breaking slice 2's already-tested,
// byte-for-byte-preserved contract: several slice-2 tests
// (ahrs_dcm_test.cpp, e.g. "drift_correction_yaw: first-ever compass
// reading...") assert that omega_i_.z updates IMMEDIATELY within the same
// drift_correction_yaw() call that produced a new yaw_error, not after a
// 5-second batch delay - switching yaw's z-contribution to route through a
// shared, batched-and-clamped sum would silently change that observable
// behavior and fail those tests, which this task explicitly requires stay
// unmodified and passing. So the two paths remain PARTIALLY separate:
//   - x/y: this slice's OWN omega_i_sum_/omega_i_sum_time_ (a real,
//     upstream-faithful 5-second batch-and-clamp, using
//     max_gyro_drift_rad_s in place of get_gyro_drift_rate() - see
//     drift_correction_accel()'s own doc comment). Slice 2 never wrote
//     x/y, so there is no existing contract to break here.
//   - z: BOTH contributors keep writing omega_i_.z directly and
//     immediately, each under its own pre-existing gate (yaw: `yaw_deltat
//     < 2.0f && spin_rate < kSpinRateLimitDeg`, unchanged from slice 2;
//     accel: `spin_rate < kSpinRateLimitDeg` alone, matching upstream's
//     own accel-half gate for _omega_I_sum, which never checked a
//     yaw_deltat). This preserves BOTH real upstream signal sources
//     feeding the z gyro-bias estimate (dropping the accel-half's z
//     contribution entirely would be a quiet loss of real upstream
//     behavior, worse than the immediate-vs-batched divergence already on
//     record) while keeping slice 2's tested contract intact. A
//     hypothetical future rework that revisits slice 2's own tests could
//     complete the unification; this slice does not, by design.
//
// kp_yaw_ SANITY CLAMP: upstream's `if (_kp_yaw < AP_AHRS_YAW_P_MIN)
// _kp_yaw.set(AP_AHRS_YAW_P_MIN)` mutates the live AP_Param in place.
// This port's kp_yaw_ is a plain float member, so the same clamp mutates
// it directly - same effect, no AP_Param involved.
//
// last_startup_ms_/use_fast_gains(): upstream's `reset(bool)` sets
// `_last_startup_ms = AP_HAL::millis()` unconditionally, on BOTH the
// public "ground start" reset AND every internal NaN-recovery reset
// check_matrix()/normalize() trigger. This port's reset() keeps its
// existing 2-argument overload (initial_accel, recover_eulers) completely
// unchanged - used as-is by the internal-recovery call sites and by all of
// slice 1's tests - and adds a 3-argument overload (+ now_ms) that ALSO
// stamps last_startup_ms_, for an explicit, deliberate top-level reset to
// call. Internal recovery resets (already documented above as very nearly
// dead code) therefore do NOT re-arm the fast-gains window the way
// upstream's do - a genuine, minor, and conservative divergence: it means
// LESS fast-gain treatment after an internal recovery, never more.
//
// PLACEHOLDER DRIFT-CORRECTION FIELD - SLICE 2's ORIGINAL NOTE (kept for
// history): omega_p_ (upstream: _omega_P) was still an always-zero private
// member in slice 2 - it belonged entirely to the then-out-of-scope accel
// half of drift_correction(). SLICE 3 UPDATE: omega_p_ is a placeholder no
// longer either - drift_correction_accel() (below) now writes it every
// GPS-triggered fusion cycle. All three of omega_p_/omega_i_/omega_yaw_p_
// are real now. matrix_update()'s math still reads all three exactly as it
// did in slice 1; nothing about matrix_update() needed to change across
// any of the three slices. reset_gyro_drift() now also zeroes
// omega_i_sum_/omega_i_sum_time_ (upstream: `_omega_I_sum.zero();
// _omega_I_sum_time = 0;`) - slice 2 dropped these because this port had
// no _omega_I_sum yet; slice 3 gives it a real (partial - x/y only, see
// the OMEGA_I_SUM UNIFICATION note above) one, so reset_gyro_drift() now
// matches upstream's full reset_gyro_drift() body again.
//
// LAST-ACCEL FALLBACK FOR INTERNAL RESETS - a genuine divergence, not just
// a naming difference: upstream's check_matrix()/normalize() call the
// PRIVATE `reset(true)` (recover_eulers) on failure, which falls through to
// a live `_ins.get_accel()` read if roll/pitch/yaw happen to be NaN.
// Because this class has no live sensor access (see above), it instead
// remembers the last `initial_accel` its own public reset() was given
// (last_initial_accel_, default zero-vector) and reuses that for the
// equivalent internal fallback. In practice this fallback path is very
// nearly dead code for both upstream and this port: update()'s real
// sequence is matrix_update -> normalize -> check_matrix -> to_euler, so by
// the time check_matrix or normalize triggers an internal reset,
// roll/pitch/yaw still hold the PREVIOUS tick's valid to_euler() output
// (to_euler for the current tick hasn't run yet) - the accel branch inside
// an internal reset(true) call fires only if roll/pitch/yaw were already
// NaN before this class ever produced a valid attitude, an even more
// degenerate case than the one renorm()/check_matrix() themselves guard
// against.
//
// BODY-FRAME TRIM ROTATION EXCLUDED FROM update(): upstream's update()
// computes `_body_dcm_matrix = _dcm_matrix * get_rotation_vehicle_body_to_
// autopilot_body()` and extracts Euler angles from THAT, not from
// `_dcm_matrix` directly. This needs a vehicle-trim-rotation concept this
// port hasn't wired in yet. update() here extracts roll/pitch/yaw directly
// from dcm_matrix instead - correct for a vehicle with zero AHRS trim, an
// approximation otherwise. Callers needing trim should apply it themselves
// once this port has a place to put it.
//
// LOGGING EXCLUDED: upstream's `#if HAL_LOGGING_ENABLED` DCM@10Hz log
// message and its GCS_SEND_TEXT watchdog-recovery message are both cut -
// no logging or GCS subsystem in this port.
//
// _renorm_val_sum/_renorm_val_count (upstream: running average of
// renorm() correction magnitude, read only by status-reporting code this
// port hasn't built) are dropped rather than ported - pure bookkeeping
// with no consumer in this slice, and cheap to re-add if a future slice's
// status-reporting work wants it.
//
// =====================================================================
// SLICE 3: drift_correction()'s ROLL/PITCH half (this addition)
// =====================================================================
//
// Upstream's drift_correction(float deltat) is dual-rate: it integrates
// accel into _ra_sum on EVERY call (once per IMU tick), but only does the
// GPS-triggered fusion work once a NEW GPS fix time is observed
// (`_gps.last_fix_time_ms() == _ra_sum_start` early-returns otherwise).
// This slice splits that into two methods matching that same split:
//   - accumulate_accel(const AccelSample&, float deltat) - the fast,
//     every-tick half (upstream: the top of drift_correction(), before its
//     `have_gps()` branch).
//   - drift_correction_accel(...) - the slow, GPS-triggered fusion half
//     (upstream: everything from the `have_gps()` branch to the end of
//     drift_correction()), named to parallel slice 2's
//     drift_correction_yaw() and distinguish it clearly.
//
// AccelSample (new struct, matching GyroSample's shape/naming exactly):
//   - delta_velocity/delta_velocity_dt: upstream's per-instance
//     get_delta_velocity() out-params (m/s, body frame / s) - sampled over
//     the sensor's own time delta specifically to avoid aliasing, per
//     upstream's own comment. Feeds ra_sum_ (upstream: _ra_sum[i]).
//   - accel: upstream's _ins.get_accel() - a SEPARATE, already-filtered
//     instantaneous body-frame reading (m/s^2), NOT derived from
//     delta_velocity/delta_velocity_dt. Feeds accel_ef (see below) and the
//     catapult-launch gain-reduction check.
//
// ACCEL-INSTANCE VOTING NOT REPRODUCED: upstream loops
// `for i in 0..get_accel_count()`, summing per-instance _ra_sum[i] and,
// at fusion time, VOTING across instances (besti/best_error/error[]/GA_b[]
// arrays) to pick whichever accelerometer's integrated reading has the
// smallest error term - deliberately exploiting different sample rates
// across accelerometers to reduce aliasing from vibration harmonics. This
// port's GyroSample already models exactly ONE gyro/accel reading per
// tick with no multi-instance array anywhere; there is nothing to vote
// BETWEEN yet, not a case of dropping real behavior for convenience. Both
// accumulate_accel() and drift_correction_accel() take exactly one
// AccelSample/one implicit instance throughout - every per-instance
// upstream construct (the `for` loop, _ra_sum[INS_MAX_INSTANCES],
// _ra_delay_buffer[INS_MAX_INSTANCES], besti, best_error, error[],
// error_dirn[], GA_b[], _active_accel_instance) collapses to its
// single-instance equivalent. A future slice modeling multiple IMUs should
// restore the voting; this slice's single-instance math is otherwise the
// complete upstream algorithm, not an approximation of it.
//
// accel_ef (upstream: `_accel_ef = _dcm_matrix * _ins.get_accel();`) IS
// NOW COMPUTED, by accumulate_accel(), every tick a real AccelSample is
// available - slice 2 left it an externally-set field because nothing in
// this class computed it yet. It STAYS a public, externally-settable field
// rather than becoming purely internal: a caller that hasn't wired up
// accumulate_accel() every tick (e.g. exercising only drift_correction_yaw()
// in isolation, as several slice-2 tests still do) can still set it
// directly, preserving slice 2's existing contract and tests unmodified.
// A caller that DOES call accumulate_accel() every tick simply doesn't
// need to touch accel_ef itself any more - it's kept in sync automatically.
//
// ra_delayed() (upstream: AP_AHRS_DCM::ra_delayed(uint8_t, const Vector3f&))
// is a trivial one-sample delay line (return the previous stored value,
// then overwrite it with the new one; "first call returns the input
// unchanged" via an is_zero() guard against a false trigger on
// initialisation) - ported near-verbatim, single-instance (see above), and
// left PUBLIC (like renorm()) rather than private, since it's a small,
// independently meaningful, independently-testable primitive in the same
// spirit as renorm().
//
// should_correct_centrifugal() (upstream:
// AP_AHRS_DCM::should_correct_centrifugal() const) is guarded
// `#if APM_BUILD_COPTER_OR_HELI || ArduSub || Blimp` with
// `return hal.util->get_soft_armed();` under those build types; Plane
// (this port's only target) falls through to upstream's own unconditional
// `return true;` at the bottom of the function - verified against the live
// upstream source, not assumed. Kept as a named static method (not inlined
// away at its one call site) so the correspondence to upstream stays
// visible and cheap to extend if this port ever grows a Copter/Sub/Blimp
// target.
//
// GpsSample EXTENDED (three new fields, all defaulted so slice 1/2's
// existing GpsSample-constructing call sites and tests keep compiling
// unchanged):
//   - velocity_ned (Vector3f): upstream `AP::gps().velocity()`, NED m/s.
//   - num_sats (uint8_t): upstream `AP::gps().num_sats()`.
//   - has_3d_fix (bool): upstream `AP::gps().status() >= AP_GPS::
//     GPS_OK_FIX_3D`. NOTE this is a DIFFERENT, stronger threshold than
//     the existing has_fix field (`status() > NO_FIX`) - checked against
//     upstream's AP_GPS::GPS_Status enum (NO_GPS=0, NO_FIX=1,
//     GPS_OK_FIX_2D=2, GPS_OK_FIX_3D=3, ...): has_fix is EXACTLY
//     `status() >= GPS_OK_FIX_2D` (there is no enum value between NO_FIX
//     and GPS_OK_FIX_2D), which is also exactly the threshold upstream's
//     catapult-launch check uses (`_gps.status() >= AP_GPS::
//     GPS_OK_FIX_2D`) - so has_fix is reused there directly, precisely,
//     not approximated.
//
// gps_gain/gps_minsats/kp (upstream: `AP_Float& gps_gain`, `AP_Int8&
// _gps_minsats`, `AP_Float& _kp`) become plain constructor fields
// (gps_gain_, gps_minsats_, kp_), defaulted to upstream's own GSCALAR
// defaults (AHRS_GPS_GAIN=1.0f, AHRS_GPS_MINSATS=6, AHRS_RP_P=0.2f) - same
// "AP_Param not wired in yet" precedent as slice 2's kp_yaw, appended to
// the existing defaulted constructor so `AhrsDcm ahrs;` and slice 2's
// `AhrsDcm ahrs(0.01f)` test call sites keep compiling unchanged. `_ki`
// (upstream: `static constexpr float _ki = 0.0087f;`, never an AP_Float
// either) stays a compile-time constant (kKi), matching kKiYaw's
// precedent.
//
// drift_correction_accel()'s remaining explicit-input parameters,
// replacing upstream singleton/subsystem reads (ADR-0012):
//   - compass/gps (CompassSample/GpsSample): the SAME structs slice 2
//     already takes. use_compass() is genuinely called a SECOND time here
//     - upstream itself calls its own (argument-free, singleton-reading)
//     use_compass() twice per full drift_correction() tick, once from
//     drift_correction_yaw() and once from here; this is faithfully
//     reproduced, not a mistake. wind_speed_ms (the scalar use_compass()
//     itself needs) is DERIVED here as `wind_estimate.xy().length()`
//     rather than taken as a second, independently-suppliable parameter -
//     upstream's use_compass() and drift_correction() both read the exact
//     same `_wind` member, so deriving one from the other here preserves
//     that single-source-of-truth relationship instead of letting a
//     caller pass two mismatched wind values.
//   - wind_estimate (Vector3f, replacing `_wind`) and airspeed_tas (float,
//     replacing `_last_airspeed_TAS`'s role as the dead-reckoning fallback
//     value, and the `#if AP_AIRSPEED_ENABLED` live-sensor override that
//     would normally take priority over it) - no wind-estimation or
//     airspeed-sensor subsystem in this port (both already out of scope
//     per slice 1/2's banner), so the caller supplies whatever estimate it
//     has, 0/zero-vector if none.
//
//     CPP-051 RE-EXAMINATION (ap-sim's SimPlane now models a REAL
//     ground-truth wind_ef - steady vector + turbulence): this parameter's
//     "no wind-estimation subsystem" reasoning is UNCHANGED by that.
//     wind_estimate here is AHRS's own ESTIMATE of wind - upstream computes
//     its real `_wind` via an EKF/DCM wind-estimation algorithm fusing
//     airspeed+GPS+heading, never by reading SITL's ground truth directly
//     (doing that would hand the estimator oracle knowledge no real flight
//     controller has). This port still has no such estimation algorithm to
//     port, so 0/zero-vector remains the correct default - this stays a
//     real, disclosed exclusion, now for "no wind ESTIMATOR exists"
//     specifically, rather than CPP-051's now-closed "no wind anywhere in
//     this port" gap.
//
//     This class still maintains its OWN
//     `_last_airspeed_TAS`-equivalent cache (last_airspeed_tas_, exposed
//     via last_airspeed_tas()), computed from GPS velocity exactly as
//     upstream does whenever GPS is available - that computation is pure
//     math needing no sensor - but, UNLIKE upstream, does not
//     automatically fall back to it when airspeed_tas is 0; a caller
//     wanting upstream's exact fallback chain can feed last_airspeed_tas()
//     back in as the next no-GPS tick's airspeed_tas argument itself,
//     making the fallback an explicit caller choice rather than an
//     implicit one.
//   - accel_healthy (bool, replacing `_ins.get_accel_health(i)`): with one
//     instance, "this instance failed its health check and was skipped"
//     collapses to "no healthy accelerometers at all" (upstream's
//     besti==-1 path) - so this parameter gates that same early return.
//   - ins_healthy (bool, replacing `_ins.healthy()`): a DIFFERENT, coarser
//     signal upstream checks separately - it zeroes the error term rather
//     than early-returning, so the P-gain/integrator code below still
//     runs (on a deliberately-zeroed error) rather than skipping entirely.
//   - now_ms: reused for use_fast_gains() (as in slice 2) AND for
//     upstream's `last_correction_time = AP_HAL::millis();` in the
//     dead-reckoning branch - both want "the current wall-clock time", so
//     one parameter serves both rather than introducing a redundant
//     second one.
//   - max_gyro_drift_rad_s (float, defaulted, replacing `AP::ins().
//     get_gyro_drift_rate()`): upstream's OWN implementation of that
//     accessor (AP_InertialSensor.h) is `return radians(0.5f/60);` - a
//     hardcoded constant with no actual per-hardware calibration behind
//     it, not a real IMU property. Defaulting this parameter to that exact
//     expression (kDefaultMaxGyroDriftRadS) reproduces upstream's real
//     behavior verbatim, not an invented approximation; a future slice
//     modeling per-hardware gyro drift rates can override it per call.
//
// EXCLUDED from drift_correction_accel(), each a genuine, named scope
// boundary rather than an oversight:
//   - Multi-accelerometer-instance voting - see the ACCEL-INSTANCE VOTING
//     note above.
//   - The position-estimate block (`_last_lat`/`_last_lng`/
//     `_position_offset_north`/`_position_offset_east`, `_have_position`) -
//     no position/GPS-location/GCS subsystem in this port; this was
//     already implicitly out of scope under slice 1's "groundspeed/
//     position...plumbing/GCS" exclusion before this slice existed, now
//     made explicit against this specific block.
//   - `_last_failure_ms` - write-only status bookkeeping upstream sets on
//     every failure path; its only reader upstream (`healthy()`, AP_AHRS_
//     DCM.cpp) is itself outside this class's current scope (no overall
//     health-status accessor exists yet). Same precedent as slice 1's
//     dropped _renorm_val_sum/_renorm_val_count. Every early-return site
//     that would have set it here just returns instead.
//   - `_active_accel_instance` - purely a multi-instance bookkeeping index,
//     meaningless with exactly one instance.
//   - The dead `#if YAW_INDEPENDENT_DRIFT_CORRECTION` block - upstream's
//     own macro guarding it is hardcoded `#define
//     YAW_INDEPENDENT_DRIFT_CORRECTION 0` immediately above the block, so
//     it is unreachable code even in upstream itself, not something this
//     port chose to cut.
//
// CATAPULT-LAUNCH GAIN REDUCTION IS INCLUDED (upstream: `if (fly_forward
// && _gps.status() >= AP_GPS::GPS_OK_FIX_2D && _gps.ground_speed() <
// GPS_SPEED_MIN && _ins.get_accel().x >= 7 && pitch > radians(-30) &&
// pitch < radians(30)) { _omega_P *= 0.5f; }`) - judged cheap and
// self-contained enough to keep rather than cut: every value it reads is
// already available (gps.has_fix stands in exactly, not approximately, for
// `status() >= GPS_OK_FIX_2D` - see the GpsSample note above; the accel.x
// reading is cached from the last accumulate_accel() call as
// last_accel_x_, matching upstream's own _ins.get_accel().x).
//
// error_rp_/get_error_rp() (upstream: `_error_rp{1.0f}`/
// `get_error_rp() const`) are ported alongside omega_p_/omega_i_ - this
// slice is what actually computes best_error/error now, so the same
// treatment slice 2 gave error_yaw_/get_error_yaw() applies here too.

#include <cfloat>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ahrs {

// upstream: #define GPS_SPEED_MIN 3 (m/s) - speed above which DCM first
// trusts GPS for a yaw lock.
inline constexpr float kGpsSpeedMinMs = 3.0f;

// upstream: #define SPIN_RATE_LIMIT 20 (deg/s) - above this, omega_I.z
// integration is disabled to avoid the DCM PI controller "getting dizzy".
inline constexpr float kSpinRateLimitDeg = 20.0f;

// upstream: AP_AHRS_YAW_P_MIN (AP_AHRS_Backend.h) - floor for kp_yaw_.
inline constexpr float kYawPMin = 0.05f;

// upstream: AP_AHRS_DCM::_ki_yaw, `static constexpr float _ki_yaw =
// 0.01f` - not an AP_Float upstream either, so no parameter here.
inline constexpr float kKiYaw = 0.01f;

// upstream: AP_AHRS_RP_P_MIN (AP_AHRS_Backend.h) - floor for kp_ (slice 3).
inline constexpr float kRpPMin = 0.05f;

// upstream: AP_AHRS_DCM::_ki, `static constexpr float _ki = 0.0087f;` -
// not an AP_Float upstream either, so no parameter here (slice 3).
inline constexpr float kKi = 0.0087f;

// upstream: #define GRAVITY_MSS 9.80665f (AP_Math/definitions.h) - slice
// 3's own local copy, matching l1_control.hpp/sim_plane.hpp's precedent of
// each file defining this constant locally rather than sharing one header.
inline constexpr float kGravityMss = 9.80665f;

// upstream: AP_InertialSensor::get_gyro_drift_rate() - see
// drift_correction_accel()'s doc comment (file banner) for why this
// hardcoded upstream return value, not a real per-hardware property, is a
// faithful default for max_gyro_drift_rad_s rather than an invented
// approximation. Not `constexpr`: math::radians() is a regular (non-
// constexpr) function - see scalar.hpp's own banner - so this is a
// runtime-initialized inline constant instead.
inline const float kDefaultMaxGyroDriftRadS = math::radians(0.5f / 60.0f);

// Everything yaw_error_compass()/use_compass()/drift_correction_yaw() need
// from the compass for one tick - see file banner.
struct CompassSample {
    math::Vector3f field;          // upstream: compass.get_field(), body-frame mag field
    float declination_rad = 0.0f;  // upstream: compass.get_declination()
    std::uint64_t last_update_usec = 0; // upstream: compass.last_update_usec()
    bool healthy = false;          // upstream: compass.use_for_yaw()
};

// Everything the same functions need from GPS for one tick - see file
// banner.
struct GpsSample {
    float ground_speed_ms = 0.0f;        // upstream: AP::gps().ground_speed()
    float ground_course_deg = 0.0f;      // upstream: AP::gps().ground_course()
    std::uint32_t last_fix_time_ms = 0;  // upstream: AP::gps().last_fix_time_ms()
    bool has_fix = false;                // upstream: AP::gps().status() > AP_GPS::NO_FIX

    // --- slice 3 additions (drift_correction_accel()) - see file banner's
    // "GpsSample EXTENDED" note. ---
    math::Vector3f velocity_ned;  // upstream: AP::gps().velocity(), NED m/s
    std::uint8_t num_sats = 0;    // upstream: AP::gps().num_sats()
    bool has_3d_fix = false;      // upstream: AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D -
                                   // a DIFFERENT, stronger threshold than has_fix above; see file banner.
};

// Everything matrix_update() needs from the primary IMU for one tick - see
// file banner. Upstream reads delta_angle/dangle_dt from
// AP_InertialSensor::get_delta_angle() (a single call returning both by
// reference plus a bool for "is this sample valid") and gyro from
// AP_InertialSensor::get_gyro(). Splitting the bool into "dangle_dt > 0
// means valid" matches upstream's OWN validity test in matrix_update() -
// it checks `dangle_dt > 0` immediately after `get_delta_angle()` returns
// true, so a caller with no valid sample this tick can simply pass
// dangle_dt = 0 rather than this struct needing a separate flag.
struct GyroSample {
    math::Vector3f delta_angle; // upstream: get_delta_angle() angle out-param, rad
    float dangle_dt = 0.0f;     // upstream: get_delta_angle() dt out-param, s; <=0 means no valid sample this tick
    math::Vector3f gyro;        // upstream: get_gyro(), rad/s
};

// Everything accumulate_accel() needs from the primary IMU for one tick -
// see file banner's SLICE 3 section. Matches GyroSample's shape/naming
// convention exactly.
struct AccelSample {
    math::Vector3f delta_velocity; // upstream: get_delta_velocity() out-param, m/s, body frame
    float delta_velocity_dt = 0.0f; // upstream: get_delta_velocity() out-param, s; <=0 means no valid sample this tick
    math::Vector3f accel;          // upstream: _ins.get_accel(), filtered instantaneous body-frame accel, m/s^2 -
                                    // a SEPARATE reading from delta_velocity/delta_velocity_dt, see file banner.
};

class AhrsDcm {
public:
    // kp_yaw: upstream `AP_Float& _kp_yaw`, defaulted to AHRS_YAW_P's
    // GSCALAR default (0.2f) - see file banner. Defaulted (not a mandatory
    // Gains struct) so slice 1's `AhrsDcm ahrs;` call sites keep compiling
    // unchanged. kp/gps_gain/gps_minsats: slice 3 additions, same
    // treatment, defaulted to upstream's own GSCALAR defaults - see file
    // banner's "gps_gain/gps_minsats/kp" note. Appended after kp_yaw so
    // slice 1/2's existing constructor call sites (including slice 2's
    // `AhrsDcm ahrs(0.01f)`) keep compiling unchanged.
    explicit AhrsDcm(float kp_yaw = 0.2f, float kp = 0.2f, float gps_gain = 1.0f, std::uint8_t gps_minsats = 6)
        // NOTE: listed in class declaration order (kp_/gps_gain_/gps_minsats_
        // precede kp_yaw_ - see the private section below) to avoid a
        // -Wreorder warning; member initialization always runs in
        // declaration order regardless of this list's textual order.
        : kp_(kp), gps_gain_(gps_gain), gps_minsats_(gps_minsats), kp_yaw_(kp_yaw) {
        dcm_matrix.identity();
    }

    AhrsDcm(const AhrsDcm&) = delete;
    AhrsDcm& operator=(const AhrsDcm&) = delete;

    // upstream: AP_AHRS_DCM::reset_gyro_drift(). Slice 2 dropped the
    // `_omega_I_sum.zero(); _omega_I_sum_time = 0;` lines because this
    // class had no _omega_I_sum equivalent yet; slice 3 gives it a real
    // (partial - x/y only, see file banner's OMEGA_I_SUM UNIFICATION note)
    // one, so this now matches upstream's full body again.
    void reset_gyro_drift() {
        omega_i_.zero();
        omega_i_sum_.zero();
        omega_i_sum_time_ = 0.0f;
    }

    // upstream: AP_AHRS_DCM::reset(bool recover_eulers), entered here with
    // an explicit initial_accel instead of a live sensor read - see file
    // banner. Reproduces upstream's remaining two branches exactly:
    //  - recover_eulers && roll/pitch/yaw already valid: rebuild dcm_matrix
    //    from them, ignoring initial_accel entirely (matches upstream).
    //  - otherwise: derive roll/pitch from the accel vector (or align flat
    //    if it's too small to trust), yaw = 0.
    void reset(const math::Vector3f& initial_accel, bool recover_eulers) {
        omega_i_.zero();
        omega_p_.zero();
        omega_yaw_p_.zero();
        omega.zero();

        last_initial_accel_ = initial_accel;

        if (recover_eulers && !std::isnan(roll) && !std::isnan(pitch) && !std::isnan(yaw)) {
            dcm_matrix.from_euler(roll, pitch, yaw);
            return;
        }

        if (initial_accel.length() > 5.0f) {
            pitch = std::atan2(initial_accel.x,
                                std::sqrt(initial_accel.y * initial_accel.y + initial_accel.z * initial_accel.z));
            roll = std::atan2(-initial_accel.y, -initial_accel.z);
        } else {
            // Accel vector too small to trust - align flat, matching
            // upstream's own fallback.
            roll = 0.0f;
            pitch = 0.0f;
        }
        dcm_matrix.from_euler(roll, pitch, 0.0f);
    }

    // 3-argument overload additionally stamping last_startup_ms_ for
    // use_fast_gains() - see file banner's "last_startup_ms_/
    // use_fast_gains()" note for why this is a separate overload rather
    // than a 3rd parameter bolted onto the one above.
    void reset(const math::Vector3f& initial_accel, bool recover_eulers, std::uint32_t now_ms) {
        reset(initial_accel, recover_eulers);
        last_startup_ms_ = now_ms;
    }

    // upstream: AP_AHRS_DCM::matrix_update(). omega_p_/omega_i_/omega_yaw_p_
    // are all real values now (slice 3 completed omega_p_/omega_i_ on top
    // of slice 2's omega_yaw_p_/omega_i_.z) - see file banner. This
    // method's own code is byte-for-byte unchanged from slice 1.
    void matrix_update(const GyroSample& sample) {
        if (sample.dangle_dt > 0.0f) {
            omega = sample.delta_angle / sample.dangle_dt;
            omega += omega_i_;
            dcm_matrix.rotate((omega + omega_p_ + omega_yaw_p_) * sample.dangle_dt);
        }

        // Re-derive omega from the (filtered) instantaneous gyro rate for
        // downstream consumers (rate controllers), matching upstream's own
        // comment: the P terms are deliberately excluded here to avoid
        // positive feedback into a future _P_gain() spin-rate calculation.
        omega = sample.gyro + omega_i_;
    }

    // upstream: AP_AHRS_DCM::renorm(). Returns false on catastrophic
    // failure (renorm_val outside [1e-6, 1e6]), matching upstream exactly;
    // the _renorm_val_sum/_renorm_val_count status bookkeeping is dropped -
    // see file banner.
    [[nodiscard]] bool renorm(const math::Vector3f& a, math::Vector3f& result) {
        const float renorm_val = 1.0f / a.length();

        if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
            if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
                return false;
            }
        }

        result = a * renorm_val;
        return true;
    }

    // upstream: AP_AHRS_DCM::normalize() - Premerlani/Bizard orthogonality
    // renormalization, equations 18-20.
    void normalize() {
        const float error = dcm_matrix.a * dcm_matrix.b; // eq.18

        const math::Vector3f t0 = dcm_matrix.a - (dcm_matrix.b * (0.5f * error)); // eq.19
        const math::Vector3f t1 = dcm_matrix.b - (dcm_matrix.a * (0.5f * error)); // eq.19
        const math::Vector3f t2 = t0 % t1;                                       // c = a x b, eq.20

        if (!renorm(t0, dcm_matrix.a) || !renorm(t1, dcm_matrix.b) || !renorm(t2, dcm_matrix.c)) {
            // Blowing up - force back to the last known-good euler angles.
            reset(last_initial_accel_, true);
        }
    }

    // upstream: AP_AHRS_DCM::check_matrix().
    void check_matrix() {
        if (dcm_matrix.is_nan()) {
            reset(last_initial_accel_, true);
            return;
        }

        // Some dcm_matrix.c.x values lead to an out-of-range asin() in
        // to_euler()'s pitch calculation - matches upstream's comment and
        // the exact 10.0 reset threshold (see Issue #20284 upstream).
        if (!(dcm_matrix.c.x < 1.0f && dcm_matrix.c.x > -1.0f)) {
            normalize();

            if (dcm_matrix.is_nan() || std::fabs(dcm_matrix.c.x) > 10.0f) {
                reset(last_initial_accel_, true);
            }
        }
    }

    // upstream: AP_AHRS_DCM::update(), reduced to this slice's scope - see
    // file banner for what's cut (drift_correction(), the body-frame trim
    // rotation, and all logging). Sequences the remaining steps in
    // upstream's own order: integrate, renormalize, paranoia-check, then
    // extract Euler angles.
    void update(const GyroSample& sample) {
        matrix_update(sample);
        normalize();
        check_matrix();
        dcm_matrix.to_euler(&roll, &pitch, &yaw);
    }

    // upstream: AP_AHRS_DCM::yaw_initialised()/get_error_yaw().
    [[nodiscard]] bool yaw_initialised() const { return have_initial_yaw_; }
    [[nodiscard]] float get_error_yaw() const { return error_yaw_; }

    // Read-only observability into the two fields this slice writes -
    // upstream has no equivalent public accessor (it reads _omega_yaw_P/
    // _omega_I directly as a member of the same class), but this port's
    // established pattern (L1Control's crosstrack_error()/
    // lateral_acceleration() etc.) exposes otherwise-private controller
    // state like this for both testing and downstream status/logging use.
    [[nodiscard]] const math::Vector3f& omega_yaw_p() const { return omega_yaw_p_; }
    [[nodiscard]] const math::Vector3f& omega_i() const { return omega_i_; }

    // upstream: AP_AHRS_DCM::_omega_P member access (upstream has no
    // dedicated accessor - it's read directly as a sibling member). Real
    // as of slice 3 - see file banner.
    [[nodiscard]] const math::Vector3f& omega_p() const { return omega_p_; }

    // upstream: AP_AHRS_DCM::get_error_rp() const. Real as of slice 3 -
    // see file banner.
    [[nodiscard]] float get_error_rp() const { return error_rp_; }

    // upstream: AP_AHRS_DCM's own `_last_airspeed_TAS` cache, exposed for
    // callers wanting to replicate upstream's dead-reckoning fallback
    // chain explicitly - see file banner's "airspeed_tas" note.
    [[nodiscard]] float last_airspeed_tas() const { return last_airspeed_tas_; }

    // upstream: AP_AHRS_DCM::_P_gain(float). Made static here - it's a
    // pure function of spin_rate alone (upstream declares it a regular,
    // non-static member despite never touching `this`).
    [[nodiscard]] static float p_gain(float spin_rate) {
        if (spin_rate < math::radians(50.0f)) {
            return 1.0f;
        }
        if (spin_rate > math::radians(500.0f)) {
            return 10.0f;
        }
        return spin_rate / math::radians(50.0f);
    }

    // upstream: AP_AHRS_DCM::_yaw_gain(). Reads accel_ef - see file banner
    // for why an unset accel_ef (default zero) makes this always return
    // its maximum, 0.9f.
    [[nodiscard]] float yaw_gain() const {
        const float vdot_ef_mag = accel_ef.xy().length();
        if (vdot_ef_mag <= 4.0f) {
            return 0.2f * (4.5f - vdot_ef_mag);
        }
        return 0.1f;
    }

    // upstream: AP_AHRS_DCM::use_fast_gains(). armed/now_ms replace
    // hal.util->get_soft_armed()/AP_HAL::millis() - see file banner.
    [[nodiscard]] bool use_fast_gains(bool armed, std::uint32_t now_ms) const {
        return !armed && (now_ms - last_startup_ms_) < 20000U;
    }

    // upstream: AP_AHRS_DCM::yaw_error_compass(Compass&). Produces a value
    // proportional to sin() of the current heading error in earth frame.
    // last_declination_/mag_earth_ are mutable caches (upstream:
    // _last_declination/_mag_earth) - only recomputed when the declination
    // actually changes, matching upstream's is_equal() guard.
    [[nodiscard]] float yaw_error_compass(const CompassSample& compass) const {
        math::Vector2f rb = dcm_matrix.mulXY(compass.field);

        if (rb.length() < FLT_EPSILON) {
            return 0.0f;
        }

        rb.normalize();
        if (rb.is_inf()) {
            // not a valid vector
            return 0.0f;
        }

        if (!math::is_equal(last_declination_, compass.declination_rad)) {
            last_declination_ = compass.declination_rad;
            mag_earth_.x = std::cos(last_declination_);
            mag_earth_.y = std::sin(last_declination_);
        }

        // Z component of the cross product of rb and mag_earth_.
        return rb % mag_earth_;
    }

    // upstream: AP_AHRS_DCM::use_compass(). Non-const: mutates
    // last_consistent_heading_ms_, matching upstream's own non-const
    // method (it mutates _last_consistent_heading the same way).
    bool use_compass(const CompassSample& compass, const GpsSample& gps, bool fly_forward,
                      bool gps_use_enabled, float wind_speed_ms, std::uint32_t now_ms) {
        if (!compass.healthy) {
            // no compass available
            return false;
        }
        if (!fly_forward || !have_gps(gps, gps_use_enabled)) {
            // we don't have any alternative to the compass
            return true;
        }
        if (gps.ground_speed_ms < kGpsSpeedMinMs) {
            // we are not going fast enough to use the GPS
            return true;
        }

        // if the current yaw differs from the GPS yaw by more than 45
        // degrees and the estimated wind speed is less than 80% of the
        // ground speed, then switch to GPS navigation. This helps prevent
        // flyaways with very bad compass offsets.
        const float error = std::fabs(math::wrap_180(math::degrees(yaw) - gps.ground_course_deg));
        if (error > 45.0f && wind_speed_ms < gps.ground_speed_ms * 0.8f) {
            if (now_ms - last_consistent_heading_ms_ > 2000U) {
                // start using the GPS for heading if the compass has been
                // inconsistent with the GPS for 2 seconds
                return false;
            }
        } else {
            last_consistent_heading_ms_ = now_ms;
        }

        // use the compass
        return true;
    }

    // upstream: AP_AHRS_DCM::drift_correction_yaw() - yaw drift correction
    // using the compass or GPS. Produces omega_yaw_p_, and contributes to
    // the omega_i_.z long-term yaw drift estimate - see file banner for
    // the dropped compass.read() re-read and the _omega_I_sum batching
    // divergence.
    void drift_correction_yaw(const CompassSample& compass, const GpsSample& gps, bool fly_forward,
                               bool armed, bool gps_use_enabled, float wind_speed_ms, std::uint32_t now_ms) {
        bool new_value = false;
        float yaw_error = 0.0f;
        float yaw_deltat = 0.0f;

        if (use_compass(compass, gps, fly_forward, gps_use_enabled, wind_speed_ms, now_ms)) {
            // we are using compass for yaw
            if (compass.last_update_usec != compass_last_update_usec_) {
                yaw_deltat = static_cast<float>(compass.last_update_usec - compass_last_update_usec_) * 1.0e-6f;
                compass_last_update_usec_ = compass.last_update_usec;

                if (!have_initial_yaw_) {
                    const float heading = calculate_heading(compass.field, compass.declination_rad);
                    dcm_matrix.from_euler(roll, pitch, heading);
                    omega_yaw_p_.zero();
                    have_initial_yaw_ = true;
                }
                new_value = true;
                yaw_error = yaw_error_compass(compass);

                // also update gps_last_update_ms_, so if we later disable
                // the compass due to significant yaw error we don't
                // suddenly change yaw with a reset
                gps_last_update_ms_ = gps.last_fix_time_ms;
            }
        } else if (fly_forward && have_gps(gps, gps_use_enabled)) {
            // we are using GPS for yaw
            if (gps.last_fix_time_ms != gps_last_update_ms_ && gps.ground_speed_ms >= kGpsSpeedMinMs) {
                yaw_deltat = static_cast<float>(gps.last_fix_time_ms - gps_last_update_ms_) * 1.0e-3f;
                gps_last_update_ms_ = gps.last_fix_time_ms;
                new_value = true;
                const float gps_course_rad = math::radians(gps.ground_course_deg);
                const float yaw_error_rad = math::wrap_PI(gps_course_rad - yaw);
                yaw_error = std::sin(yaw_error_rad);

                // reset yaw to match GPS heading under any of upstream's 3
                // conditions: never had yaw before, 20s+ stale, or a large
                // error at high speed (see upstream's own comment for the
                // exact rationale of each).
                if (!have_initial_yaw_ || yaw_deltat > 20.0f ||
                    (gps.ground_speed_ms >= 3.0f * kGpsSpeedMinMs && std::fabs(yaw_error_rad) >= 1.047f)) {
                    dcm_matrix.from_euler(roll, pitch, gps_course_rad);
                    omega_yaw_p_.zero();
                    have_initial_yaw_ = true;
                    yaw_error = 0.0f;
                }
            }
        }

        if (!new_value) {
            // we don't have any new yaw information - slowly decay
            // omega_yaw_p_ to cope with loss of our yaw source
            omega_yaw_p_ *= 0.97f;
            return;
        }

        // convert the error vector to body frame
        const float error_z = dcm_matrix.c.z * yaw_error;

        // the spin rate changes the P gain, and disables the integration
        // at higher rates
        const float spin_rate = omega.length();

        // sanity check kp_yaw_ - see file banner's "kp_yaw_ SANITY CLAMP" note
        if (kp_yaw_ < kYawPMin) {
            kp_yaw_ = kYawPMin;
        }

        omega_yaw_p_.z = error_z * p_gain(spin_rate) * kp_yaw_ * yaw_gain();
        if (use_fast_gains(armed, now_ms)) {
            omega_yaw_p_.z *= 8.0f;
        }

        // don't update the drift term if we lost the yaw reference for
        // more than 2 seconds - see file banner's _omega_I_sum note for
        // why this accumulates directly into omega_i_.z rather than
        // through a batched-and-clamped intermediate sum.
        if (yaw_deltat < 2.0f && spin_rate < math::radians(kSpinRateLimitDeg)) {
            omega_i_.z += error_z * kKiYaw * yaw_deltat;
        }

        error_yaw_ = 0.8f * error_yaw_ + 0.2f * std::fabs(yaw_error);
    }

    // upstream: AP_AHRS_DCM::ra_delayed(uint8_t instance, const Vector3f&
    // ra) - a one-sample delay line matching the GPS's inherent lag when
    // comparing against the accel-integrated gravity estimate.
    // Single-instance (see file banner's ACCEL-INSTANCE VOTING note):
    // upstream indexes _ra_delay_buffer[instance], this port has exactly
    // one buffer. Left public, like renorm(), as an independently
    // meaningful and independently-testable primitive.
    [[nodiscard]] math::Vector3f ra_delayed(const math::Vector3f& ra) {
        // get the old element, and then fill it with the new element
        const math::Vector3f ret = ra_delay_buffer_;
        ra_delay_buffer_ = ra;
        if (ret.is_zero()) {
            // use the current vector if the previous vector is exactly
            // zero - prevents an error on initialisation, matching
            // upstream exactly.
            return ra;
        }
        return ret;
    }

    // upstream: AP_AHRS_DCM::should_correct_centrifugal() const - see file
    // banner for why Plane (this port's only target) reduces to an
    // unconditional true.
    [[nodiscard]] static bool should_correct_centrifugal() { return true; }

    // upstream: the per-tick accel-accumulation code at the TOP of
    // AP_AHRS_DCM::drift_correction(float deltat), executed every call
    // regardless of GPS rate - see file banner's dual-rate note. Builds up
    // ra_sum_ (the earth-frame accel-vs-time integral consumed by
    // drift_correction_accel() once a new GPS fix arrives) and refreshes
    // accel_ef for yaw_gain(). Multi-accelerometer-instance voting is not
    // reproduced - see file banner.
    void accumulate_accel(const AccelSample& sample, float deltat) {
        if (sample.delta_velocity_dt > 0.0f) {
            // by using delta_velocity/delta_velocity_dt instead of
            // sample.accel, the accel value is sampled over the right
            // time delta for this sensor, which prevents an aliasing
            // effect - matches upstream's own comment.
            const math::Vector3f accel_ef_body = sample.delta_velocity / sample.delta_velocity_dt;
            const math::Vector3f accel_ef_earth = dcm_matrix * accel_ef_body;
            // integrate the accel vector in the earth frame between GPS readings
            ra_sum_ += accel_ef_earth * deltat;
        }

        // set accel_ef based on the filtered accel - upstream:
        // `_accel_ef = _dcm_matrix * _ins.get_accel();`. See file banner.
        accel_ef = dcm_matrix * sample.accel;
        last_accel_x_ = sample.accel.x; // feeds the catapult-launch check below

        // keep a sum of the deltat values, so we know how much time we
        // have integrated over
        ra_deltat_ += deltat;
    }

    // upstream: AP_AHRS_DCM::drift_correction(float deltat)'s GPS-triggered
    // fusion half - everything from its `have_gps()` branch to its end.
    // Named to parallel slice 2's drift_correction_yaw() - see file
    // banner's SLICE 3 section for every parameter's provenance and every
    // exclusion.
    void drift_correction_accel(const CompassSample& compass, const GpsSample& gps, bool fly_forward, bool armed,
                                 bool gps_use_enabled, const math::Vector3f& wind_estimate, float airspeed_tas,
                                 bool accel_healthy, bool ins_healthy, std::uint32_t now_ms,
                                 float max_gyro_drift_rad_s = kDefaultMaxGyroDriftRadS) {
        math::Vector3f velocity;
        // Zero-initialized defensively (unlike upstream's bare local) so
        // no compiler's flow analysis can flag a false-positive
        // maybe-uninitialized warning across the if/else below - every
        // real path through it assigns this before it's ever read.
        std::uint32_t last_correction_time = 0;

        if (!have_gps(gps, gps_use_enabled) || !gps.has_3d_fix || gps.num_sats < gps_minsats_) {
            // no GPS, or not a good lock. From experience upstream needs
            // at least 6 satellites to get a really reliable velocity
            // number from the GPS.
            //
            // As a fallback we use the fixed wing acceleration correction
            // if the caller has an airspeed estimate (which upstream only
            // has if fly_forward is set), otherwise no correction.
            if (ra_deltat_ < 0.2f) {
                // not enough time has accumulated
                return;
            }

            // use airspeed to estimate our ground velocity in earth frame
            // by subtracting the wind
            velocity = dcm_matrix.colx() * airspeed_tas;
            velocity += wind_estimate;

            last_correction_time = now_ms;
            have_gps_lock_ = false;
        } else {
            if (gps.last_fix_time_ms == ra_sum_start_) {
                // we don't have a new GPS fix - nothing more to do
                return;
            }
            velocity = gps.velocity_ned;
            last_correction_time = gps.last_fix_time_ms;
            if (!have_gps_lock_) {
                // if we didn't have GPS lock in the last drift correction
                // interval then set the velocities equal
                last_velocity_ = velocity;
            }
            have_gps_lock_ = true;

            // keep last airspeed estimate for dead-reckoning purposes
            math::Vector3f airspeed = velocity - wind_estimate;
            // rotate vector to body frame - upstream: _body_dcm_matrix
            // (this port's update() extracts eulers straight from
            // dcm_matrix with no separate body-trim matrix - see file
            // banner's update() note).
            airspeed = dcm_matrix.mul_transpose(airspeed);
            // take positive component in X direction - mimics a pitot tube
            last_airspeed_tas_ = std::fmax(airspeed.x, 0.0f);
        }

        // upstream's position-estimate block (_last_lat/_last_lng/
        // _position_offset_north/_position_offset_east) is excluded - see
        // file banner.

        // see if this is our first time through - in which case we just
        // set up the start times and return
        if (ra_sum_start_ == 0) {
            ra_sum_start_ = last_correction_time;
            last_velocity_ = velocity;
            return;
        }

        // equation 9: get the corrected acceleration vector in earth
        // frame. Units are m/s/s
        math::Vector3f ga_e(0.0f, 0.0f, -1.0f);

        if (ra_deltat_ <= 0.0f) {
            // waiting for more data
            return;
        }

        bool using_gps_corrections = false;
        const float ra_scale = 1.0f / (ra_deltat_ * kGravityMss);

        if (should_correct_centrifugal() && (have_gps_lock_ || fly_forward)) {
            const float v_scale = gps_gain_ * ra_scale;
            const math::Vector3f vdelta = (velocity - last_velocity_) * v_scale;
            ga_e += vdelta;
            ga_e.normalize();
            if (ga_e.is_inf()) {
                // wait for some non-zero acceleration information
                return;
            }
            using_gps_corrections = true;
        }

        // Multi-accelerometer-instance voting collapses to a single
        // instance gated by accel_healthy - see file banner.
        if (!accel_healthy) {
            // no healthy accelerometers
            return;
        }

        ra_sum_ *= ra_scale;

        // get the delayed ra_sum to match the GPS lag
        math::Vector3f ga_b = using_gps_corrections ? ra_delayed(ra_sum_) : ra_sum_;
        if (ga_b.is_zero()) {
            // wait for some non-zero acceleration information
            return;
        }
        ga_b.normalize();
        if (ga_b.is_inf()) {
            // wait for some non-zero acceleration information
            return;
        }

        math::Vector3f error = ga_b % ga_e;
        // take dot product to catch case vectors are opposite sign and parallel
        const float error_dirn = ga_b * ga_e;
        float best_error = error.length();
        // catch case where orientation is 180 degrees out
        if (error_dirn < 0.0f) {
            best_error = 1.0f;
        }

        // to reduce the impact of two competing yaw controllers, we reduce
        // the impact of the gps/accelerometers on yaw when we are flat,
        // but still allow for yaw correction using the accelerometers at
        // high roll angles as long as we have a GPS. upstream calls
        // use_compass() a SECOND time here - see file banner.
        if (use_compass(compass, gps, fly_forward, gps_use_enabled, wind_estimate.xy().length(), now_ms)) {
            if (have_gps(gps, gps_use_enabled) && math::is_equal(gps_gain_, 1.0f)) {
                error.z *= std::sin(std::fabs(roll));
            } else {
                error.z = 0.0f;
            }
        }

        // if ins is unhealthy then stop attitude drift correction and hope
        // the gyros are OK for a while. Just slowly reduce omega_p_ to
        // prevent previous bad accels from throwing us off.
        if (!ins_healthy) {
            error.zero();
        } else {
            // convert the error term to body frame
            error = dcm_matrix.mul_transpose(error);
        }

        if (error.is_nan() || error.is_inf()) {
            // don't allow bad values
            check_matrix();
            return;
        }

        error_rp_ = 0.8f * error_rp_ + 0.2f * best_error;

        // base the P gain on the spin rate
        const float spin_rate = omega.length();

        // sanity check kp_ - see file banner's "kp_yaw_ SANITY CLAMP" note
        // (drift_correction_yaw()) for the equivalent treatment here.
        if (kp_ < kRpPMin) {
            kp_ = kRpPMin;
        }

        // we now want to calculate omega_p_ - the value that drags us
        // quickly to the accelerometer reading.
        omega_p_ = error * p_gain(spin_rate) * kp_;
        if (use_fast_gains(armed, now_ms)) {
            omega_p_ *= 8.0f;
        }

        // catapult-launch gain reduction - see file banner. gps.has_fix
        // stands in exactly for `status() >= GPS_OK_FIX_2D`.
        if (fly_forward && gps.has_fix && gps.ground_speed_ms < kGpsSpeedMinMs && last_accel_x_ >= 7.0f &&
            pitch > math::radians(-30.0f) && pitch < math::radians(30.0f)) {
            // assume we are in a launch acceleration, and reduce the rp
            // gain by 50% to reduce the impact of GPS lag on takeoff
            // attitude when using a catapult
            omega_p_ *= 0.5f;
        }

        // accumulate some integrator error. See file banner's OMEGA_I_SUM
        // UNIFICATION note: x/y go through the batched-and-clamped
        // omega_i_sum_/omega_i_sum_time_ machinery; z is folded into
        // omega_i_.z immediately instead, preserving slice 2's
        // already-tested, unbatched z contract.
        if (spin_rate < math::radians(kSpinRateLimitDeg)) {
            omega_i_sum_.x += error.x * kKi * ra_deltat_;
            omega_i_sum_.y += error.y * kKi * ra_deltat_;
            omega_i_.z += error.z * kKi * ra_deltat_;
            omega_i_sum_time_ += ra_deltat_;
        }

        if (omega_i_sum_time_ >= 5.0f) {
            // limit the rate of change of omega_i_ to the (explicit)
            // max_gyro_drift_rad_s parameter - see file banner. This
            // ensures short term errors don't cause a buildup of omega_i_
            // beyond the physical limits of the device.
            const float change_limit = max_gyro_drift_rad_s * omega_i_sum_time_;
            omega_i_sum_.x = math::constrain_value(omega_i_sum_.x, -change_limit, change_limit);
            omega_i_sum_.y = math::constrain_value(omega_i_sum_.y, -change_limit, change_limit);
            omega_i_.x += omega_i_sum_.x;
            omega_i_.y += omega_i_sum_.y;
            omega_i_sum_.zero();
            omega_i_sum_time_ = 0.0f;
        }

        // zero our accumulator ready for the next GPS step
        ra_sum_.zero();
        ra_deltat_ = 0.0f;
        ra_sum_start_ = last_correction_time;

        // remember the velocity for next time
        last_velocity_ = velocity;
    }

    // CPP-078: single-call orchestration wrapper collapsing the four
    // methods above (update(), accumulate_accel(), drift_correction_yaw(),
    // drift_correction_accel()) into the ONE call point a future generic
    // "run one estimator cycle" abstraction needs. Exists purely so
    // modules/ap-vehicle/include/fwcpp/vehicle/mode.hpp's tick()
    // (~line 908-946) can stop calling four AhrsDcm-specific methods
    // directly - none of which have any equivalent on EkfCore (this
    // port's NavEKF3-equivalent estimator, which fuses via
    // fuse_gps_velocity()/fuse_magnetometer()/etc. instead, not these
    // DCM-specific steps). A real polymorphic AhrsBackend-style interface
    // (implemented by both AhrsDcm and a future EkfCore adapter) is the
    // actual next step this groundwork exists for - deliberately NOT
    // attempted here (CPP-078 scope: collapse four known-sequential
    // calls into one, nothing more).
    //
    // Pure, mechanical pass-through: changes nothing about how the four
    // calls below work internally, and calls them in the exact order,
    // with the exact arguments, mode.hpp's real call site already used
    // before this method existed - re-verified directly against that
    // call site (not assumed) before writing this. Parameter list is the
    // union of what those four calls collectively need, in the order
    // they're consumed below:
    //   - gyro_sample: update()'s only argument.
    //   - accel_sample, dt: accumulate_accel()'s two arguments.
    //   - compass, gps: shared by drift_correction_yaw() and
    //     drift_correction_accel().
    //   - fly_forward, armed_and_safety_off, gps_use_enabled, now_ms:
    //     shared by both drift_correction_*() calls (named
    //     armed_and_safety_off, not armed, to match the real call site's
    //     own local variable name - plane.is_armed_and_safety_off()).
    //   - wind_speed_ms: drift_correction_yaw()-only.
    //   - wind_estimate, airspeed_tas, accel_healthy, ins_healthy:
    //     drift_correction_accel()-only.
    // drift_correction_accel()'s own trailing max_gyro_drift_rad_s
    // parameter is deliberately NOT exposed here: mode.hpp's real call
    // site never passes it either, so before and after this refactor it
    // resolves to the same kDefaultMaxGyroDriftRadS default either way.
    void update_full_cycle(const GyroSample& gyro_sample, const AccelSample& accel_sample, float dt,
                            const CompassSample& compass, const GpsSample& gps, bool fly_forward,
                            bool armed_and_safety_off, bool gps_use_enabled, float wind_speed_ms,
                            const math::Vector3f& wind_estimate, float airspeed_tas, bool accel_healthy,
                            bool ins_healthy, std::uint32_t now_ms) {
        update(gyro_sample);
        accumulate_accel(accel_sample, dt);
        drift_correction_yaw(compass, gps, fly_forward, armed_and_safety_off, gps_use_enabled, wind_speed_ms, now_ms);
        drift_correction_accel(compass, gps, fly_forward, armed_and_safety_off, gps_use_enabled, wind_estimate,
                                airspeed_tas, accel_healthy, ins_healthy, now_ms);
    }

    // Primary attitude representation - upstream: _dcm_matrix.
    math::Matrix3f dcm_matrix;

    // Euler angles extracted from dcm_matrix, radians - upstream: roll/pitch/yaw.
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // Corrected gyro vector for downstream consumers (e.g. rate
    // controllers) - upstream: _omega.
    math::Vector3f omega;

    // Earth-frame accelerometer reading feeding yaw_gain()'s observability
    // gain - upstream: _accel_ef. Written every tick by accumulate_accel()
    // (slice 3) but remains a public, externally-settable field for
    // callers that don't call accumulate_accel() every tick - see file
    // banner's "accel_ef IS NOW COMPUTED" note.
    math::Vector3f accel_ef;

private:
    // Real as of slice 3 (drift_correction_accel()) - see file banner.
    math::Vector3f omega_p_; // upstream: _omega_P

    // Real, slice-2/slice-3-written values now - see file banner.
    math::Vector3f omega_i_;     // upstream: _omega_I
    math::Vector3f omega_yaw_p_; // upstream: _omega_yaw_P

    // drift_correction_accel()'s x/y integrator batch (slice 3) - upstream:
    // _omega_I_sum/_omega_I_sum_time. z is NOT accumulated here - see file
    // banner's OMEGA_I_SUM UNIFICATION note for why it stays a separate,
    // immediate contribution into omega_i_.z instead.
    math::Vector3f omega_i_sum_;
    float omega_i_sum_time_ = 0.0f;

    // drift_correction_accel()'s bookkeeping (slice 3) - upstream:
    // _ra_sum[instance], _ra_deltat, _ra_sum_start, _ra_delay_buffer[instance],
    // _have_gps_lock, _last_velocity, _last_airspeed_TAS, _error_rp{1.0f},
    // and _ins.get_accel().x (cached here as last_accel_x_ for the
    // catapult-launch check) - all collapsed to single-instance where
    // upstream indexed by accelerometer instance (see file banner).
    math::Vector3f ra_sum_;
    float ra_deltat_ = 0.0f;
    std::uint32_t ra_sum_start_ = 0;
    math::Vector3f ra_delay_buffer_;
    bool have_gps_lock_ = false;
    math::Vector3f last_velocity_;
    float last_airspeed_tas_ = 0.0f;
    float last_accel_x_ = 0.0f;
    float error_rp_ = 1.0f;

    // kp_/gps_gain_/gps_minsats_: see file banner's "gps_gain/gps_minsats/
    // kp" note. kp_ is mutated in place by drift_correction_accel()'s
    // sanity clamp, matching upstream's `_kp.set(...)`.
    float kp_ = 0.2f;
    float gps_gain_ = 1.0f;
    std::uint8_t gps_minsats_ = 6;

    // See file banner's "LAST-ACCEL FALLBACK FOR INTERNAL RESETS" note.
    math::Vector3f last_initial_accel_;

    // kp_yaw_: see file banner. Mutated in place by drift_correction_yaw()'s
    // sanity clamp, matching upstream's `_kp_yaw.set(...)`.
    float kp_yaw_ = 0.2f;

    // yaw_error_compass()'s declination cache - mutable because the method
    // is logically const (a pure function of its CompassSample argument
    // and current attitude) even though it memoizes cos/sin internally.
    mutable float last_declination_ = 0.0f;
    mutable math::Vector2f mag_earth_{1.0f, 0.0f}; // upstream: _mag_earth{1, 0}

    // drift_correction_yaw()'s bookkeeping - upstream: _compass_last_update,
    // _gps_last_update, have_initial_yaw, _error_yaw{1.0f}.
    std::uint64_t compass_last_update_usec_ = 0;
    std::uint32_t gps_last_update_ms_ = 0;
    bool have_initial_yaw_ = false;
    float error_yaw_ = 1.0f;

    // use_compass()'s 2-second-inconsistency hysteresis latch - upstream:
    // _last_consistent_heading.
    std::uint32_t last_consistent_heading_ms_ = 0;

    // use_fast_gains()'s window start - upstream: _last_startup_ms. Only
    // stamped by the 3-argument reset() overload - see file banner.
    std::uint32_t last_startup_ms_ = 0;

    // upstream: AP_AHRS_DCM::have_gps() - `_gps_use == GPSUse::Disable ||
    // AP::gps().status() <= AP_GPS::NO_FIX` collapses to this, per
    // GpsSample/gps_use_enabled's own collapsed representations.
    [[nodiscard]] bool have_gps(const GpsSample& gps, bool gps_use_enabled) const {
        return gps_use_enabled && gps.has_fix;
    }

    // upstream: Compass::calculate_heading(const Matrix3f&, uint8_t) - see
    // file banner for why this one call site's real math is ported
    // directly rather than stubbed behind a pre-computed-heading parameter.
    [[nodiscard]] float calculate_heading(const math::Vector3f& field, float declination_rad) const {
        const float cos_pitch_sq = 1.0f - (dcm_matrix.c.x * dcm_matrix.c.x);

        // Tilt compensated magnetic field Y component:
        const float head_y = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

        // Tilt compensated magnetic field X component:
        const float head_x =
            field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

        // Added constrain to keep bad values from ruining DCM yaw, matching upstream.
        const float heading =
            math::constrain_value(std::atan2(-head_y, head_x), -static_cast<float>(M_PI), static_cast<float>(M_PI));

        return math::wrap_PI(heading + declination_rad);
    }
};

} // namespace fwcpp::ahrs
