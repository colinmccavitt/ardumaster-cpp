#pragma once

// AhrsBackend: abstract interface covering exactly the `ahrs` surface
// Plane (modules/ap-vehicle/include/fwcpp/vehicle/plane.hpp, mode.hpp)
// actually depends on today. CPP-079, phase 2 of vehicle-integration prep
// (CPP-078, phase 1, already collapsed Plane's four separate per-tick
// AhrsDcm calls into one update_full_cycle()).
//
// THE REAL SURFACE, VERIFIED DIRECTLY - NOT the ticket's own initial
// survey: the CPP-079 ticket that scoped this work carried a pre-verified
// list of "method calls" (airspeed_EAS(), cos_pitch(), get_accel_ef(),
// get_relative_position_D_home(), get_yaw_rate_earth(), groundspeed(),
// groundspeed_vector(), head_wind(), home_is_locked(), set_fly_forward(),
// set_home(), using_airspeed_sensor(), plus accumulate_accel()/update()/
// update_full_cycle()), produced by:
//
//   grep -oE 'ahrs\.[a-zA-Z_]+\(' plane.hpp mode.hpp | sed 's/^[^:]*://' | sort -u
//
// Re-running that grep and then manually classifying EVERY hit by whether
// the match falls inside a `//` comment (the ticket's own instruction was
// "re-verify this exact list directly, don't trust it as complete" - this
// is exactly the case where that mattered) shows that airspeed_EAS(),
// cos_pitch(), get_accel_ef(), get_relative_position_D_home(),
// get_yaw_rate_earth(), groundspeed(), groundspeed_vector(), head_wind(),
// home_is_locked(), set_fly_forward(), set_home(), using_airspeed_sensor(),
// accumulate_accel(), and update() ONLY EVER appear inside comments
// describing what UPSTREAM ArduPilot's AP_AHRS does - mostly explaining
// why THIS port's much simpler ahrs has no equivalent subsystem to call
// (e.g. "no wind-estimation subsystem in this port" near head_wind();
// "no airspeed-sensor subsystem" near using_airspeed_sensor()). Not one of
// them is a real, live call on this port's `ahrs` member anywhere in
// plane.hpp or mode.hpp.
//
// The one and only REAL method call, found the same way, is:
//   modules/ap-vehicle/include/fwcpp/vehicle/mode.hpp:962
//   plane.ahrs.update_full_cycle(gyro_sample, in.accel_sample, in.dt,
//                                 compass, gps_sample, plane.fly_forward(), ...)
//
// and the real DATA MEMBER accesses (re-verified the same comment-vs-code
// way, via `grep -oE 'ahrs\.[a-zA-Z_]+' ...` minus the method-call list)
// are exactly the six the ticket also named: roll, pitch, yaw (plain
// floats), omega (whole Vector3f, plus .x/.y/.z), dcm_matrix (whole
// Matrix3f, plus .c and .transposed()), accel_ef (whole Vector3f, plus
// .z) - live at plane.hpp lines 5140/5141/5219/5220/5235/5271/5278/5279/
// 5300/5397/5414/5415/5537/5538/5539/5892-5894/5899/5900 and mode.hpp
// line 212 (`plane_.ahrs.yaw`).
//
// So the real, complete interface surface is dramatically smaller than
// the ticket's own uncross-checked survey suggested: ONE method plus SIX
// getters, not eighteen methods. Reported explicitly here, per the
// ticket's own allowance for finding the real surface different from
// expected - in this case cleaner and narrower, not messier - rather than
// silently forcing an eighteen-method abstraction that no real call site
// would ever exercise (most of upstream's AP_AHRS surface simply has no
// equivalent subsystem in this port at all: no wind estimator, no
// position/home subsystem, no airspeed sensor, no GCS/logging).
//
// GETTERS VS FIELD ACCESS (the ticket's central, explicitly-required
// design decision): resolved as (a) - AhrsDcm (ahrs_dcm.hpp) gets six new
// getter methods (get_roll()/get_pitch()/get_yaw()/get_omega()/
// get_dcm_matrix()/get_accel_ef()) that simply return the existing public
// fields, UNCHANGED. Plane's own ~22 real call sites reading those fields
// directly are LEFT AS-IS, NOT converted to go through the new getters.
//
// Rationale: Plane's `ahrs` member is not becoming polymorphic in this
// ticket (explicitly out of scope - see below), so nothing today ever
// calls these getters through an AhrsBackend reference/pointer - the
// getters exist purely to give the interface something legal to declare
// against a private-by-convention field. Converting Plane's ~22 call
// sites now would mean touching a much larger, functionally unrelated
// surface (StabilizeInputs-building code, telemetry-snapshot code,
// ground-steering code, L1 input-building code, roll/pitch-limit scaling)
// purely for stylistic consistency with an interface nothing yet
// exercises - real regression risk (every converted line needs its own
// re-verification against plane.hpp's existing, already-passing behavior)
// for zero present behavioral or interface-satisfaction benefit. This is
// a deliberately different call from CPP-078's own precedent of updating
// mode.hpp's call site: there, updating the call site WAS the entire
// point (proving Plane could actually use the new collapsed entry point
// in real, live code). Here, nothing in Plane uses AhrsBackend
// polymorphically yet, so there is no analogous forcing function - only
// speculative churn. Once a future ticket makes Plane's `ahrs` member
// genuinely polymorphic, converting those ~22 call sites to getters
// becomes both NECESSARY (an AhrsBackend reference has no public fields
// to read directly) and LOW-RISK to do at the same time (a real
// compile-time forcing function catches every site, and it lands as one
// coherent diff instead of a speculative one made now, ahead of need).
// Until then, this ticket keeps the getters purely additive and Plane
// completely untouched - matching the ticket's explicit "no change to
// Plane at all" scope.
//
// AhrsDcm implements this interface with ZERO behavior change - see
// ahrs_dcm.hpp's own class declaration (`class AhrsDcm : public
// AhrsBackend`): every interface method is either a pre-existing AhrsDcm
// method reused as an override with no code change (update_full_cycle(),
// CPP-078) or a brand new one-line getter returning a pre-existing public
// field verbatim. Proven by tests/ahrs_dcm_test.cpp's "CPP-079:
// AhrsBackend interface" section, matching CPP-078's own A/B-test
// precedent: an `AhrsBackend&` bound to a real AhrsDcm instance is driven
// through single-tick and multi-tick (60-tick) scenarios, and every value
// read back through the interface is compared byte-identical (==, not
// Catch::Approx) against the same value read directly off the concrete
// AhrsDcm instance.
//
// NOT ATTEMPTED HERE - named explicitly, per the ticket's own scope
// boundary, as two separate, deliberately deferred future steps:
//   - Making Plane's own `ahrs` member polymorphic (a concrete AhrsDcm
//     field today -> some AhrsBackend-referencing form) - a separate,
//     later, riskier step touching plane.hpp's member declaration and
//     every one of the ~22 real call sites named above (and requiring the
//     getter-conversion work described above).
//   - An EkfCore adapter implementing this interface - EkfCore (this
//     port's NavEKF3-equivalent estimator, modules/ap-ekf) has no
//     equivalent at all of home_is_locked()/groundspeed()/DCM-specific
//     concepts (dcm_matrix, the DCM-specific drift-correction gains), and
//     would need real, substantial conversion logic from its own
//     quaternion/position/velocity state to roll/pitch/yaw/dcm_matrix/
//     accel_ef - a genuine adapter, not attempted here.
//
// Forward-declares (rather than includes) AhrsDcm's GyroSample/
// AccelSample/CompassSample/GpsSample structs: this header only ever uses
// them by const reference in a pure-virtual declaration, which needs no
// complete type, and forward declaration here keeps this interface header
// free of any dependency on ahrs_dcm.hpp - ahrs_dcm.hpp includes THIS
// header (to derive `AhrsDcm : public AhrsBackend`), so the reverse
// direction would be circular.
//
// Matches this port's existing polymorphic-base-class idiom (see
// modules/ap-vehicle/include/fwcpp/vehicle/plane.hpp's `class Mode`: a
// virtual destructor, deleted copy ctor/assignment, pure virtual methods
// marked `= 0`, `[[nodiscard]]` on const accessors) - the only other
// pure-virtual interface in this codebase at the time this was written.

#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ahrs {

// Forward declarations only - see file banner. Full definitions live in
// ahrs_dcm.hpp.
struct GyroSample;
struct AccelSample;
struct CompassSample;
struct GpsSample;

class AhrsBackend {
public:
    AhrsBackend() = default;
    virtual ~AhrsBackend() = default;
    AhrsBackend(const AhrsBackend&) = delete;
    AhrsBackend& operator=(const AhrsBackend&) = delete;

    // The one real per-tick entry point mode.hpp's tick() actually calls
    // (mode.hpp:962) - see AhrsDcm::update_full_cycle()'s own doc comment
    // (ahrs_dcm.hpp, CPP-078) for this parameter list's full provenance.
    virtual void update_full_cycle(const GyroSample& gyro_sample, const AccelSample& accel_sample, float dt,
                                    const CompassSample& compass, const GpsSample& gps, bool fly_forward,
                                    bool armed_and_safety_off, bool gps_use_enabled, float wind_speed_ms,
                                    const math::Vector3f& wind_estimate, float airspeed_tas, bool accel_healthy,
                                    bool ins_healthy, std::uint32_t now_ms) = 0;

    // Getters standing in for Plane's six real direct-field-access
    // patterns (roll/pitch/yaw/omega/dcm_matrix/accel_ef) - see file
    // banner's "GETTERS VS FIELD ACCESS" note for why these are new,
    // additive interface methods rather than a removal of the underlying
    // public fields, and why Plane's own call sites are not converted to
    // use them in this ticket.
    [[nodiscard]] virtual float get_roll() const = 0;
    [[nodiscard]] virtual float get_pitch() const = 0;
    [[nodiscard]] virtual float get_yaw() const = 0;
    [[nodiscard]] virtual const math::Vector3f& get_omega() const = 0;
    [[nodiscard]] virtual const math::Matrix3f& get_dcm_matrix() const = 0;
    [[nodiscard]] virtual const math::Vector3f& get_accel_ef() const = 0;
};

} // namespace fwcpp::ahrs
