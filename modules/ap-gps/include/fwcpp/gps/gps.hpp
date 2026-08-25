#pragma once

// Port of AP_GPS's SITL backend - NOT the full generic multi-backend AP_GPS
// interface (dozens of protocol backends: UBLOX, NMEA, SBP, MAVLink, DroneCAN,
// ...) - just what SITL's own backend actually does, matching this port's
// established "match the real backend, not the generic interface"
// methodology (ap-hal's RcInput/RcOutput/AnalogIn/UartDriver). CPP-033.
//
// Upstream (Plane-4.7.0, read directly from the pinned worktree, not from
// training-data memory):
//   - AP_GPS/AP_GPS_SITL.h (40 lines, in full).
//   - AP_GPS/AP_GPS_SITL.cpp (120 lines, in full).
//   - AP_GPS/GPS_Backend.cpp's velocity_to_speed_course() (~line 110-114, 4
//     lines) - ported byte-for-byte below.
//
// WHY THIS MODULE EXISTS: CPP-028 ported AhrsDcm's full drift-correction
// algorithm (drift_correction_yaw()/drift_correction_accel()), but nothing
// in this port could ever call it with real GPS data - there was no GPS
// module at all. This module is that missing input source, existing solely
// to fill in fwcpp::ahrs::GpsSample (ap-ahrs/ahrs_dcm.hpp) - it does not
// invent a new output type, it builds the thing that produces the type
// that already exists and is already consumed by tested code.
//
// AP_GPS_SITL::read()'s REAL behavior, faithfully reproduced below:
//   - Rate-limited to at most once per 200ms: `if (now - last_update_ms <
//     200) return false;` - a call inside that window is a genuine no-op,
//     matching real GPS receiver update rates (5Hz), not a port-invented
//     throttle.
//   - Reads SITL's OWN GROUND-TRUTH STATE DIRECTLY (sitl->state.latitude/
//     longitude/altitude/speedN/speedE/speedD) - NO noise, NO simulated GPS
//     error model of any kind. This is a real, faithful fact about
//     upstream's own SITL GPS backend, not this port cutting a corner -
//     verified by reading AP_GPS_SITL.cpp in full, not assumed.
//   - Produces state.status = AP_GPS::GPS_OK_FIX_3D and state.num_sats = 15
//     UNCONDITIONALLY, on every successful read, with no branch that could
//     ever produce anything else. SITL's GPS backend never simulates a
//     degraded fix. Faithfully reproduced as an unconditional assignment
//     below, not invented leniency.
//
// EXCLUDED - each a genuine, named scope boundary, not an oversight:
//   - Time-of-week (gps_time()'s simulation_timeval()/epoch arithmetic,
//     state.time_week/time_week_ms) - out of scope per this module's own
//     ticket: nothing in fwcpp::ahrs::GpsSample needs GPS time-of-week, only
//     wall-clock last_fix_time_ms, which this module's caller already
//     supplies via now_ms (matching every other explicit-context module in
//     this port, ADR-0012).
//   - HDOP/VDOP (state.hdop/vdop, hardcoded to 100 upstream) - GpsSample has
//     no such field and no consumer needs it.
//   - Accuracy fields (have_speed_accuracy/have_horizontal_accuracy/
//     have_vertical_accuracy - upstream sets only the "have" bools, never
//     the actual accuracy values, which stay commented out in
//     AP_GPS_SITL.cpp itself) - no consumer.
//   - Position/Location (state.location, built from sitl->state.latitude/
//     longitude/altitude via Location::AltFrame::ABSOLUTE) - traced every
//     real consumer of fwcpp::ahrs::GpsSample (drift_correction_yaw(),
//     drift_correction_accel(), use_compass(), have_gps() - ahrs_dcm.hpp)
//     to confirm this before writing a line of code: NONE of them read
//     position/lat/lon/altitude, only ground_speed_ms/ground_course_deg/
//     last_fix_time_ms/has_fix/velocity_ned/num_sats/has_3d_fix. This port
//     has no Location/geodesy machinery at all (no position-estimate block
//     anywhere - already excluded from drift_correction_accel() itself per
//     ahrs_dcm.hpp's own file banner), and none is needed here either. A
//     GPS module that only needs to feed AhrsDcm never needs lat/lon.
//   - Multi-instance/blending - AP_GPS's own multi-backend voting/blending
//     across GPS instances (primary-instance selection, blending weights)
//     has no equivalent here: this port models exactly one GPS, matching
//     GpsSample's own single-instance shape (same precedent as AhrsDcm's
//     own single-accelerometer-instance collapse, see its file banner's
//     "ACCEL-INSTANCE VOTING NOT REPRODUCED" note).
//   - All non-SITL backends (UBLOX, NMEA, SBP, MAVLink, DroneCAN, ...) -
//     SITL is this port's only runtime target (house rule: SITL is the
//     runtime, not an excuse to simplify - but it IS the one and only real
//     backend this port ever needs to match).
//
// NO SINGLETONS, EXPLICIT INPUT INSTEAD (ADR-0012): update() takes the true
// NED velocity directly as an explicit parameter rather than reading a
// SimPlane& or any other sensor-truth source - upstream's own `AP::sitl()
// ->state.speedN/speedE/speedD` singleton read becomes this module's
// `true_velocity_ned` parameter. This module has NO dependency on ap-sim -
// same "test-only dependency" separation already established for
// ap-vehicle (ap-vehicle's own vehicle_test.cpp links ap-sim for its
// closed-loop tests; the ap-vehicle library itself does not). now_ms
// REPLACES AP_HAL::millis() - the same explicit-clock treatment every other
// module in this port already receives.
//
// velocity_to_speed_course() (GPS_Backend.cpp) ported byte-for-byte:
// ground_course = wrap_360(degrees(atan2f(velocity.y, velocity.x))),
// ground_speed = velocity.xy().length(). Vector3::xy() (ap-math, added for
// AhrsDcm's yaw_gain()) is reused here rather than duplicating the x/y
// length computation.
//
// TYPE CHOICE - fwcpp::ahrs::GpsSample IS THE OUTPUT TYPE, UNCHANGED: this
// module does not move GpsSample out of ap-ahrs into some more "neutral"
// location, even though GpsSample is logically GPS-owned data. GpsSample is
// already deeply embedded in ahrs_dcm.hpp's own tests/file-banner/consumers
// (drift_correction_yaw()/drift_correction_accel()/use_compass()/
// have_gps()), and this module's only real need is the TYPE itself, not
// ownership of it - moving it would touch ahrs_dcm.hpp's includes and every
// existing ahrs_dcm_test.cpp GpsSample-constructing test for no material
// benefit. Depending on ap-ahrs (and transitively ap-math, via ap-ahrs) is
// the smaller, more honest change.
//
// SANITY CHECK ON THE RATE LIMIT'S STARTING CONDITION: last_update_ms_
// starts at 0 (matching upstream's uninitialized-but-effectively-zero
// `uint32_t last_update_ms;` for an object constructed at boot). This means
// - exactly matching upstream - the FIRST successful update only happens
// once now_ms reaches 200 (`now_ms - 0 < 200` is true, i.e. gated, for
// every now_ms in [0, 199]; at now_ms == 200, `200 - 0 < 200` is false, so
// the update proceeds). A caller ticking from now_ms == 0 will see has_fix
// stay false (GpsSample's own default) for the first ~200ms of wall-clock
// time, exactly like real GPS hardware acquiring its first fix - not a
// port quirk.

#include <cmath>
#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::gps {

class Gps {
public:
    // upstream: AP_GPS_SITL::read(). Rate-limited to at most once per
    // 200ms, matching upstream's `now - last_update_ms < 200` early return
    // exactly (see file banner's "SANITY CHECK" note for the first-call
    // boundary). A call inside the 200ms window is a genuine no-op - the
    // previously-computed sample() is left completely unchanged, same as
    // upstream's read() returning false without touching `state` at all.
    void update(const math::Vector3f& true_velocity_ned, std::uint32_t now_ms) {
        if (now_ms - last_update_ms_ < 200U) {
            return;
        }
        last_update_ms_ = now_ms;

        // upstream: state.velocity.x/y/z = speedN/speedE/speedD (SITL's own
        // ground-truth NED velocity, no noise - see file banner).
        sample_.velocity_ned = true_velocity_ned;

        // upstream: velocity_to_speed_course(state) (GPS_Backend.cpp),
        // ported byte-for-byte.
        sample_.ground_course_deg =
            math::wrap_360(math::degrees(std::atan2(true_velocity_ned.y, true_velocity_ned.x)));
        sample_.ground_speed_ms = true_velocity_ned.xy().length();

        // upstream: state.status = AP_GPS::GPS_OK_FIX_3D; state.num_sats =
        // 15; - both unconditional on every successful read, see file
        // banner. has_fix collapses `status() > AP_GPS::NO_FIX` (GpsSample's
        // own established meaning, ahrs_dcm.hpp) - true whenever this class
        // has ever completed an update, exactly matching an always-3D-fix
        // backend.
        sample_.num_sats = 15;
        sample_.has_3d_fix = true;
        sample_.has_fix = true;

        // upstream: state.last_gps_time_ms = now;
        sample_.last_fix_time_ms = now_ms;
    }

    // Current GPS state as an fwcpp::ahrs::GpsSample - see file banner's
    // "TYPE CHOICE" note. Default-constructed (has_fix=false, everything
    // else zero) until the first successful update().
    [[nodiscard]] const ahrs::GpsSample& sample() const { return sample_; }

private:
    ahrs::GpsSample sample_;
    std::uint32_t last_update_ms_ = 0;
};

} // namespace fwcpp::gps
