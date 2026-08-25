#pragma once

// Port of AP_AHRS_DCM's pure gyro-integration attitude core: matrix_update,
// reset, check_matrix, renorm, normalize, and a reduced update() that
// sequences them. CPP-028, slice 1. Upstream: AP_AHRS/AP_AHRS_DCM.h,
// AP_AHRS_DCM.cpp (Plane-4.7.0) - read directly from the pinned upstream
// worktree, not from training-data memory.
//
// NO DRIFT CORRECTION IN THIS SLICE: drift_correction()/drift_correction_yaw()
// /yaw_error_compass() (GPS-velocity and compass-yaw fusion, ~450 of
// AP_AHRS_DCM.cpp's ~1340 lines - by far the largest excluded chunk),
// estimate_wind()/set_external_wind_estimate()/wind_estimate() (needs
// airspeed + GPS), airspeed_EAS() (both overloads) and
// get_unconstrained_airspeed_EAS() (needs an airspeed sensor),
// groundspeed_vector() (needs GPS), get_location()/get_origin()/
// get_relative_position_*_origin() (needs GPS-derived position + Location
// integration), healthy()/pre_arm_check()/send_ekf_status_report()/
// yaw_source_available()/get_control_limits() (status/arming plumbing tied
// to the broader AP_AHRS_Backend interface and GCS, neither of which exist
// in this port), _P_gain()/_yaw_gain()/use_fast_gains()/have_gps()/
// should_correct_centrifugal() (only ever called from drift_correction(),
// so out of scope with it), and backup_attitude() (writes
// hal.util->persistent_data for watchdog-reset recovery - no watchdog
// subsystem in this port) are all deliberately NOT ported. A future slice
// 2 adds drift correction once this port has GPS/Compass subsystems to
// fuse - see this file's bottom-of-header note on what that slice needs.
//
// This class produces a real, gyro-only attitude estimate that slowly
// drifts without correction - exactly the upstream algorithm with the
// correction terms omitted, not a simplified approximation of it.
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
// PLACEHOLDER DRIFT-CORRECTION FIELDS: omega_i_/omega_p_/omega_yaw_p_ exist
// as always-zero private members (upstream: _omega_I/_omega_P/_omega_yaw_P)
// rather than being pruned. matrix_update()'s math reads them exactly as
// upstream does; a future drift-correction slice writes them and nothing
// about matrix_update() needs to change. reset_gyro_drift() is kept too
// (it's a trivial one-liner upstream even including drift correction -
// `_omega_I.zero()`; this slice drops upstream's accompanying
// `_omega_I_sum.zero(); _omega_I_sum_time = 0;` since _omega_I_sum/
// _omega_I_sum_time are drift_correction()-only bookkeeping this slice
// never populates in the first place).
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

#include <cmath>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ahrs {

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
    AhrsDcm() { dcm_matrix.identity(); }

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

    // upstream: AP_AHRS_DCM::matrix_update(). No _omega_P/_omega_yaw_P/
    // _omega_I contribution beyond the always-zero placeholders in this
    // slice - see file banner.
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

    // Primary attitude representation - upstream: _dcm_matrix.
    math::Matrix3f dcm_matrix;

    // Euler angles extracted from dcm_matrix, radians - upstream: roll/pitch/yaw.
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // Corrected gyro vector for downstream consumers (e.g. rate
    // controllers) - upstream: _omega.
    math::Vector3f omega;

private:
    // Always-zero placeholders in this slice - see file banner.
    math::Vector3f omega_i_;     // upstream: _omega_I
    math::Vector3f omega_p_;     // upstream: _omega_P
    math::Vector3f omega_yaw_p_; // upstream: _omega_yaw_P

    // See file banner's "LAST-ACCEL FALLBACK FOR INTERNAL RESETS" note.
    math::Vector3f last_initial_accel_;
};

} // namespace fwcpp::ahrs

// SLICE 2 NOTE (drift correction, once GPS/Compass exist in this port):
// upstream's drift_correction(deltat) fuses a delayed accel-vs-gravity
// error into _omega_P (roll/pitch) and _omega_I (integrated gyro bias),
// gated by have_gps()/use_fast_gains()/_P_gain(spin_rate), plus
// drift_correction_yaw() feeding _omega_yaw_P from either GPS-velocity
// heading (have_gps()) or yaw_error_compass(compass) otherwise. Both need
// an explicit-inputs struct of their own (GPS velocity/fix status, compass
// field vector + declination, plus the AP_Float gain parameters _kp/_kp_yaw/
// gps_gain/beta this class doesn't own today) - the same GyroSample-style
// treatment used here, sized up for two more subsystems' worth of inputs.
// matrix_update() itself needs no changes: it already reads omega_p_/
// omega_yaw_p_/omega_i_ every tick, they're just always zero until slice 2
// starts writing them.
