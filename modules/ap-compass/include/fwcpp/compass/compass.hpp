#pragma once

// Port of AP_Compass_SITL's real backend - NOT the full generic multi-
// instance AP_Compass interface (dozens of I2C/SPI hardware backends,
// calibration, consistency checks) - just what SITL's own backend
// actually does, honestly reduced to what this port can compute without a
// geodesy subsystem it doesn't have. Matches this port's established
// "match the real backend, not the generic interface" methodology
// (ap-hal's RcInput/RcOutput/AnalogIn/UartDriver, ap-gps's Gps). CPP-XXX.
//
// Upstream (Plane-4.7.0, read directly from the pinned worktree, not from
// training-data memory):
//   - AP_Compass/AP_Compass_SITL.h (44 lines, in full) / .cpp (150 lines,
//     in full).
//   - SITL/SIM_Aircraft.cpp's Aircraft::update_mag_field_bf() (~line 211,
//     36 lines, in full) - THIS is where the real body-frame field
//     actually gets computed; AP_Compass_SITL itself only reads
//     `_sitl->state.bodyMagField` (a value update_mag_field_bf() already
//     wrote) and adds sensor-simulation noise/delay/calibration error on
//     top. Verified directly by reading both files, not assumed.
//   - AP_Declination/AP_Declination.h (44 lines) / .cpp (146 lines) -
//     checked just enough to confirm its shape (a 19x37 lat/lon-indexed
//     World Magnetic Model lookup table, `get_mag_field_ef(lat, lng, ...)`)
//     - NOT ported, see "PERMANENT SCOPE BOUNDARY" below.
//
// update_mag_field_bf()'s REAL computation, faithfully reproduced below
// (minus the two exclusions named there):
//   AP_Declination::get_mag_field_ef(lat, lng, intensity, declination,
//   inclination);                      // Gauss, degrees, degrees
//   Vector3f mag_ef(1e3f * intensity, 0, 0);   // milliGauss, pure "North"
//   Matrix3f R;
//   R.from_euler(0, -radians(inclination), radians(declination));
//   mag_ef = R * mag_ef;                       // rotate to earth-frame NED
//   ... (magnetic anomaly perturbation - excluded, see below) ...
//   mag_bf = dcm.transposed() * mag_ef;        // true attitude -> body frame
//   ... (motor interference - excluded, see below) ...
//
// PERMANENT SCOPE BOUNDARY - REAL-WORLD LAT/LON-KEYED DECLINATION IS OUT
// OF SCOPE, NOT A FUTURE-SLICE PLACEHOLDER:
// AP_Declination::get_mag_field_ef() is a REAL-WORLD GEOMAGNETIC MODEL
// (a tabulated World Magnetic Model sampling, `declination_table`/
// `inclination_table`/`intensity_table`, each a 19 (latitude) x 37
// (longitude) float array spanning +-90 lat / +-180 lon in 10-degree
// steps, bilinearly interpolated) keyed by REAL WGS-84 latitude/longitude.
// This port has NO real latitude/longitude anywhere - confirmed directly:
// ap-common/include/fwcpp/location.hpp's own file banner documents
// Location's lat/lng fields as a FLAT LOCAL-TANGENT-PLANE representation
// (north/east meters relative to an arbitrary local origin, reused as
// fixed-point "degrees" purely for API-shape compatibility with upstream's
// Location type - never real geodesy), and ap-gps/gps.hpp's own file
// banner independently confirms the same thing from GPS's side (GpsSample
// has no position/lat/lon field at all, because nothing in this port ever
// had real coordinates to put there). There is therefore no latitude/
// longitude value ANYWHERE in this port's data model that could be handed
// to a real WMM lookup and mean anything - unlike this port's other
// "not yet built" exclusions (e.g. a future INS/EKF subsystem), there is
// no future slice that closes this gap without first inventing real
// geodesy from scratch, which is its own separate, much larger effort
// outside every ticket this port has cut so far. This is the SAME
// treatment already established repeatedly this session for a "real,
// location-dependent quantity this port can't compute": Tecs's fixed
// EAS2TAS default, SimPlane's fixed sea-level air density, AhrsDcm's
// explicit wind-estimate-as-input. The substitute here is the same shape:
// a FIXED, caller-configurable earth-frame field (declination/inclination/
// intensity), not a real lookup.
//
// FIXED EARTH-FIELD DEFAULT - A CITED REAL-WORLD VALUE, NOT AN INVENTED
// ONE: Halifax, Nova Scotia, Canada (44 deg 39' N, 63 deg 36' W - a real
// mid-latitude Northern Hemisphere location, not an arbitrary number).
// Declination = -16.1 deg (16.1 deg W), Inclination = 66.3 deg, Intensity
// = 51011 nT (= 510.11 mGauss, since 1 nT = 0.01 mGauss) - values from the
// R `oce` oceanographic package's own documented `magneticField()` worked
// example (https://dankelley.github.io/oce/reference/magneticField.html,
// `magneticField(-(63 + 36/60), 44 + 39/60, Sys.Date())`), itself computed
// from a real WMM/IGRF geomagnetic model, not invented for this port.
// Any real mid-latitude location would do equally well for this port's
// purposes (this is a REPRESENTATIVE default, not a claim that this
// vehicle is actually in Halifax) - Halifax was picked because a
// maintained, independent piece of software already publishes and tests
// against this exact numeric example, making the citation independently
// verifiable rather than a number typed from memory.
//
// EXCLUDED - each a genuine, named scope boundary, not an oversight:
//   - Real-world lat/lon-keyed declination/inclination/intensity
//     (AP_Declination) - PERMANENT scope boundary, see above.
//   - Sensor noise (`rand_vec3f() * _sitl->mag_noise`), the 50-entry delay
//     ring buffer (`_sitl->mag_delay`), and calibration imperfection
//     (elliptical diagonal/off-diagonal correction matrix inversion,
//     `mag_ofs`/`mag_diag`/`mag_offdiag`/`mag_scaling`/`mag_orient`) - none
//     of AP_Compass_SITL's own sensor-simulation-imperfection layer is
//     modeled. This port's compass reading is the true field, exactly,
//     always - the same "no noise model" treatment already established
//     for RcInput/AnalogIn.
//   - The magnetic anomaly perturbation (`mag_anomaly_ned`/
//     `mag_anomaly_hgt`, an inverse-cube-scaled perturbation vs. height
//     above ground) and motor interference (`mag_mot * battery_current`) -
//     no anomaly-injection or battery-current subsystem in this port.
//   - Multi-instance compasses, external-vs-internal distinction,
//     compass calibration/consistency checks, COMPASS_CAL_ENABLED,
//     `mag_fail` (no-data/frozen-compass failure injection) - single fixed
//     compass model only, always healthy when driven (see WHO CALLS
//     update() below).
//
// WHO COMPUTES THE TRUE BODY-FRAME FIELD, Compass OR THE CALLER? THE
// CALLER - Compass::update() TAKES AN ALREADY-ROTATED BODY-FRAME FIELD:
// AP_Compass_SITL::_timer() itself never touches attitude at all - it
// reads `_sitl->state.bodyMagField`, a value SITL's OWN Aircraft physics
// class (update_mag_field_bf(), SIM_Aircraft.cpp) already rotated into
// body frame using the aircraft's TRUE dcm, before the compass driver
// ever sees it. The compass driver's real job is strictly narrower than
// "know the attitude and rotate a field" - it is "report whatever body-
// frame field the airframe's physics truthfully produced". This port
// reproduces that division of labor exactly: Compass::update() takes the
// pre-rotated body-frame field as an explicit parameter (matching this
// port's GyroSample/AccelSample/Gps::update(true_velocity_ned, ...)
// pattern of "whoever holds ground truth supplies it" - ADR-0012, no
// singletons), and rotate_earth_field_to_body() below is a SEPARATE,
// clearly-named helper for whichever caller DOES hold true attitude (a
// test/SITL-integration harness, mirroring exactly how the closed-loop
// tests already rotate SimPlane::dcm for other sensors) to compute that
// body-frame field from this class's own fixed earth field. Compass
// itself never reads attitude, estimated or true - reading AhrsDcm's own
// ESTIMATED attitude to produce a "sensor" reading would be circular (a
// real magnetometer is not derived from the attitude estimator it exists
// to correct), and reading a live SimPlane& would resurrect the singleton
// pattern ADR-0012 forbids. See StabilizeInputs::compass_field_bf/
// compass_healthy's own doc comments (plane.hpp) for how tick() (mode.hpp)
// wires this in production - it cannot supply true attitude either, and
// doesn't need to: a caller (production hardware driver, or this test
// harness) is expected to supply the already-body-frame field directly,
// exactly as it already supplies true_velocity_ned/accel_sample.
//
// TYPE CHOICE - fwcpp::ahrs::CompassSample IS THE OUTPUT TYPE, UNCHANGED:
// same rationale as ap-gps/gps.hpp's own "TYPE CHOICE" note - CompassSample
// is already deeply embedded in ahrs_dcm.hpp's own tests/file-banner/
// consumers (drift_correction_yaw()/yaw_error_compass()/use_compass()),
// and this module's only real need is the TYPE itself, not ownership of
// it. Depending on ap-ahrs (and transitively ap-math) is the smaller,
// more honest change - exactly ap-gps's own precedent.
//
// "HEALTHY" MEANS "HAS update() EVER BEEN CALLED", PERMANENTLY, LIKE GPS's
// OWN has_fix - NOT A PER-TICK VALIDITY FLAG: update() unconditionally
// sets healthy=true, matching AP_Compass_SITL's own `mag_fail == 0` (no
// failure injection - see EXCLUDED above) always-successful `_timer()`
// path. Whether a GIVEN tick's compass_field_bf is meaningful is entirely
// the CALLER's decision (StabilizeInputs::compass_healthy, plane.hpp) -
// this class does not second-guess it.
#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::compass {

// See file banner's "FIXED EARTH-FIELD DEFAULT" note - Halifax, Nova
// Scotia, Canada (44.65 deg N, 63.6 deg W), from the `oce` R package's own
// documented magneticField() example. Not `constexpr`: math::radians() is
// a regular (non-constexpr) runtime function - see scalar.hpp's own
// banner and ahrs_dcm.hpp's kDefaultMaxGyroDriftRadS precedent - so these
// are runtime-initialized inline constants instead.
inline const float kDefaultDeclinationRad = math::radians(-16.1f);
inline const float kDefaultInclinationRad = math::radians(66.3f);
// milliGauss - matches AP_Compass_SITL's own "units are milli-Gauss"
// comment and CompassSample::field's own real-compass-driver-output units.
// 51011 nT * 0.01 (mGauss per nT) = 510.11 mGauss.
inline constexpr float kDefaultIntensityMilligauss = 510.11f;

class Compass {
public:
    // Builds the fixed earth-frame field from declination/inclination/
    // intensity, upstream's OWN exact construction (update_mag_field_bf(),
    // see file banner) reproduced byte-for-byte:
    // mag_ef = Matrix3f::from_euler(0, -inclination, declination) *
    //          Vector3f(intensity, 0, 0)
    // Defaulted to kDefaultDeclinationRad/kDefaultInclinationRad/
    // kDefaultIntensityMilligauss so a caller needing no customization
    // (every test below but the "custom earth field" ones, and Plane's own
    // default `compass::Compass compass;` member) keeps compiling
    // unchanged with a real, cited value rather than an arbitrary one.
    explicit Compass(float declination_rad = kDefaultDeclinationRad, float inclination_rad = kDefaultInclinationRad,
                      float intensity_milligauss = kDefaultIntensityMilligauss)
        : declination_rad_(declination_rad) {
        math::Matrix3f r;
        r.from_euler(0.0f, -inclination_rad, declination_rad);
        mag_ef_ = r * math::Vector3f(intensity_milligauss, 0.0f, 0.0f);
    }

    // TEST/SITL-INTEGRATION-HARNESS HELPER, NOT CALLED BY update() OR BY
    // ANY PRODUCTION tick() PATH - see file banner's "WHO COMPUTES..."
    // note. Rotates this Compass's own fixed earth-frame field into body
    // frame given the vehicle's TRUE attitude, upstream's exact
    // `dcm.transposed() * mag_ef` line (update_mag_field_bf(),
    // SIM_Aircraft.cpp) reproduced directly. A caller with true attitude
    // (e.g. a closed-loop test's SimPlane::dcm) uses this to build the
    // body-frame field update() actually wants.
    [[nodiscard]] math::Vector3f rotate_earth_field_to_body(const math::Matrix3f& true_dcm) const {
        return true_dcm.transposed() * mag_ef_;
    }

    // upstream: AP_Compass_SITL::_timer()'s net effect (accumulate_sample()
    // + drain_accumulated_samples(), minus every exclusion in the file
    // banner) - the compass "reads" whatever body-frame field it's handed.
    // Takes an ALREADY BODY-FRAME field, not true attitude - see file
    // banner. Always marks the sample healthy - see file banner's
    // "HEALTHY MEANS..." note; a caller that should not treat this tick's
    // reading as valid simply does not call update() this tick (see
    // StabilizeInputs::compass_healthy, plane.hpp).
    void update(const math::Vector3f& field_bf, std::uint64_t now_usec) {
        sample_.field = field_bf;
        sample_.declination_rad = declination_rad_;
        sample_.last_update_usec = now_usec;
        sample_.healthy = true;
    }

    // Current compass state as an fwcpp::ahrs::CompassSample - see file
    // banner's "TYPE CHOICE" note. Default-constructed (healthy=false,
    // field/declination_rad/last_update_usec all zero) until the first
    // update() call.
    [[nodiscard]] const ahrs::CompassSample& sample() const { return sample_; }

    // This Compass's own fixed earth-frame field - exposed for tests that
    // verify the declination/inclination/intensity construction directly
    // (compass_test.cpp), and for any caller that wants the earth-frame
    // vector itself rather than a body-frame rotation of it.
    [[nodiscard]] const math::Vector3f& earth_field() const { return mag_ef_; }

    // This Compass's own fixed declination - exposed so a caller/test can
    // confirm it independently of a sample() (which only carries it after
    // the first update()).
    [[nodiscard]] float declination_rad() const { return declination_rad_; }

private:
    float declination_rad_;
    math::Vector3f mag_ef_;
    ahrs::CompassSample sample_;
};

} // namespace fwcpp::compass
