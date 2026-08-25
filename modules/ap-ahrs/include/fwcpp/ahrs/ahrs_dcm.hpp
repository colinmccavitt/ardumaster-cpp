#pragma once

// Port of AP_AHRS_DCM's gyro-integration attitude core (slice 1) plus YAW
// drift correction (slice 2, this addition): drift_correction_yaw() and
// everything it calls. CPP-028. Upstream: AP_AHRS/AP_AHRS_DCM.h,
// AP_AHRS_DCM.cpp (Plane-4.7.0) - read directly from the pinned upstream
// worktree, not from training-data memory.
//
// SLICE 1 (matrix_update/normalize/check_matrix/renorm/reset/update, no
// drift correction at all - commit b0e2e6d) left omega_i_/omega_p_/
// omega_yaw_p_ as always-zero placeholder fields that matrix_update()
// already read every tick. SLICE 2 (this addition) starts writing
// omega_yaw_p_ and omega_i_.z from real compass/GPS yaw fusion.
// matrix_update() ITSELF HAS NOT CHANGED - not one line - it already
// consumed these exact fields in slice 1.
//
// STILL NOT PORTED, and now the single biggest remaining chunk of
// AP_AHRS_DCM.cpp: AP_AHRS_DCM::drift_correction(float deltat) itself
// (minus the drift_correction_yaw() call already inside it) - the
// roll/pitch half of drift correction. It fuses a GPS-lag-delayed,
// multi-accelerometer-instance-voted accel-vs-gravity error into
// omega_P and omega_I.x/omega_I.y, gated by should_correct_centrifugal()
// and wind estimation - needing a GPS-lag delay ring buffer
// (_ra_delay_buffer/ra_delayed), multi-IMU-instance voting, and wind
// estimation this port hasn't built. A future slice 3. Everything else
// slice 1's banner already excluded for the same reasons (wind estimation,
// airspeed, groundspeed/position, status/arming plumbing/GCS,
// backup_attitude()/watchdog persistence) is still excluded.
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
// This class produces a real, gyro-only attitude estimate that still
// slowly drifts on the roll/pitch axes without slice 3's accel correction
// - exactly the upstream algorithm with those correction terms omitted,
// not a simplified approximation of it. Yaw no longer drifts unbounded:
// it is now corrected exactly as upstream does, from compass or GPS.
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
// _omega_I_sum BATCHING IS NOT REPRODUCED - a genuine divergence, not an
// oversight: upstream accumulates this slice's yaw contribution into a
// SHARED `_omega_I_sum.z` (also written by the out-of-scope accel-based
// drift_correction(), same vector), only actually folded into `_omega_I`
// every 5 seconds (`_omega_I_sum_time >= 5`), constrained by
// `AP::ins().get_gyro_drift_rate()` - an IMU property this port hasn't
// wired in, and inseparable from drift_correction()'s multi-instance accel
// voting that owns the sum's other two axes. Since that batching/clamping
// machinery cannot be ported without the accel half, this slice instead
// accumulates the yaw contribution directly and continuously into
// omega_i_.z on every gated tick - the identical `error_z * kKiYaw *
// yaw_deltat` term upstream computes, just applied immediately rather than
// batched-then-clamped. A future slice 3 porting drift_correction()'s
// accel half should revisit this once omega_i_.z has a real batching
// partner for x/y again.
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
// PLACEHOLDER DRIFT-CORRECTION FIELD: omega_p_ (upstream: _omega_P) is
// still an always-zero private member - it belongs entirely to the
// out-of-scope accel half of drift_correction() (slice 3). omega_i_/
// omega_yaw_p_ are placeholders no longer: this slice writes both.
// matrix_update()'s math still reads all three exactly as it did in slice
// 1; nothing about matrix_update() needed to change for any of this.
// reset_gyro_drift() is kept too (it's a trivial one-liner upstream even
// including drift correction - `_omega_I.zero()`; this slice still drops
// upstream's accompanying `_omega_I_sum.zero(); _omega_I_sum_time = 0;`
// since this port has no _omega_I_sum to zero - see the batching note
// above).
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

class AhrsDcm {
public:
    // kp_yaw: upstream `AP_Float& _kp_yaw`, defaulted to AHRS_YAW_P's
    // GSCALAR default (0.2f) - see file banner. Defaulted (not a mandatory
    // Gains struct) so slice 1's `AhrsDcm ahrs;` call sites keep compiling
    // unchanged.
    explicit AhrsDcm(float kp_yaw = 0.2f) : kp_yaw_(kp_yaw) { dcm_matrix.identity(); }

    AhrsDcm(const AhrsDcm&) = delete;
    AhrsDcm& operator=(const AhrsDcm&) = delete;

    // upstream: AP_AHRS_DCM::reset_gyro_drift() - see file banner for why
    // _omega_I_sum/_omega_I_sum_time aren't reproduced.
    void reset_gyro_drift() { omega_i_.zero(); }

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

    // upstream: AP_AHRS_DCM::matrix_update(). omega_p_ is still an
    // always-zero placeholder (slice 3); omega_i_/omega_yaw_p_ are real,
    // slice-2-written values now - see file banner. This method's own code
    // is byte-for-byte unchanged from slice 1.
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
    // gain - upstream: _accel_ef. See file banner for why this is a plain
    // settable field rather than something this class computes.
    math::Vector3f accel_ef;

private:
    // Always-zero placeholder in this slice (accel half of drift
    // correction, slice 3) - see file banner.
    math::Vector3f omega_p_; // upstream: _omega_P

    // Real, slice-2-written values now - see file banner.
    math::Vector3f omega_i_;     // upstream: _omega_I
    math::Vector3f omega_yaw_p_; // upstream: _omega_yaw_P

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
