// Tests for fwcpp::ekf::EkfCoreBackend (CPP-080) - the adapter making
// EkfCore implement fwcpp::ahrs::AhrsBackend. See fwcpp/ekf/ekf_backend.hpp
// for the full design writeup (caching rationale, which sample feeds
// get_omega()/get_accel_ef(), the bias-correction convention verified
// directly against update_strapdown_equations_ned(), and the
// fusion-gating decision).
//
// Per the ticket's own acceptance criteria, every check here derives its
// "expected" value INDEPENDENTLY of the getter formula under test - never
// by re-invoking the same EkfCoreBackend/EkfCore code path a second time
// and asserting it agrees with itself:
//   - get_dcm_matrix()/get_roll()/get_pitch()/get_yaw(): compared against
//     a fresh math::Matrix3f built via Matrix3f::from_euler() (a
//     completely separate code path from QuaternionT::rotation_matrix())
//     and/or plain scalar arithmetic on the known commanded rotation -
//     never against QuaternionF::to_euler()/rotation_matrix() called a
//     second time on the SAME quaternion object the implementation used.
//   - get_omega()/get_accel_ef(): the bias-correction arithmetic
//     (raw_rate - state.bias/dt_ekf_avg) is redone with plain floats in
//     the test itself, using EkfCore's own real, verified convention (see
//     ekf_backend.hpp's "BIAS-CORRECTION CONVENTION" note, itself derived
//     directly from update_strapdown_equations_ned()'s real body, not
//     assumed).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ahrs/ahrs_backend.hpp>
#include <fwcpp/ekf/ekf_backend.hpp>

namespace ekf = fwcpp::ekf;
namespace ahrs = fwcpp::ahrs;
namespace math = fwcpp::math;

namespace {
constexpr float kGravity = 9.80665f; // AP_Math/definitions.h GRAVITY_MSS - same literal every test file in this port uses.
}

// ============================================================================
// EkfCoreBackend must genuinely implement AhrsBackend, not merely compile
// against it via some other route - matches ahrs_dcm_test.cpp's own
// identical static_assert for AhrsDcm (CPP-079).
// ============================================================================
static_assert(std::is_base_of_v<ahrs::AhrsBackend, ekf::EkfCoreBackend>,
              "EkfCoreBackend must implement AhrsBackend - see ekf_backend.hpp");

TEST_CASE("EkfCoreBackend: a truly stationary, level vehicle reports identity attitude, zero rate, "
          "and accel_ef.z == -GRAVITY_MSS",
          "[ekf_backend]") {
    // The ticket's own explicitly-required sanity check, taken completely
    // literally: no rotation commanded at all (delta_angle/gyro both
    // zero), accel reading exactly cancels gravity in body frame (the
    // level, stationary case) - matching ekf_core_test.cpp's own
    // "truly stationary" test 3 precedent for what a stationary sample
    // looks like for this port's IMU sample convention.
    ekf::EkfCoreBackend backend;

    ahrs::GyroSample gyro; // delta_angle/gyro both default to zero
    gyro.dangle_dt = 0.01f;

    ahrs::AccelSample accel;
    accel.accel = math::Vector3f(0.0f, 0.0f, -kGravity);
    accel.delta_velocity = accel.accel * 0.01f;
    accel.delta_velocity_dt = 0.01f;

    backend.update_full_cycle(gyro, accel, 0.01f, ahrs::CompassSample{}, ahrs::GpsSample{}, /*fly_forward=*/false,
                               /*armed_and_safety_off=*/false, /*gps_use_enabled=*/false, /*wind_speed_ms=*/0.0f,
                               math::Vector3f{}, /*airspeed_tas=*/0.0f, /*accel_healthy=*/false,
                               /*ins_healthy=*/false, /*now_ms=*/0);

    REQUIRE(backend.get_roll() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(backend.get_pitch() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(backend.get_yaw() == Catch::Approx(0.0f).margin(1e-6));

    const math::Matrix3f identity_ref = [] {
        math::Matrix3f m;
        m.from_euler(0.0f, 0.0f, 0.0f);
        return m;
    }();
    REQUIRE(backend.get_dcm_matrix().a == identity_ref.a);
    REQUIRE(backend.get_dcm_matrix().b == identity_ref.b);
    REQUIRE(backend.get_dcm_matrix().c == identity_ref.c);

    REQUIRE(backend.get_omega().x == Catch::Approx(0.0f).margin(1e-9));
    REQUIRE(backend.get_omega().y == Catch::Approx(0.0f).margin(1e-9));
    REQUIRE(backend.get_omega().z == Catch::Approx(0.0f).margin(1e-9));

    // The ticket's own literal requirement.
    REQUIRE(backend.get_accel_ef().x == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(backend.get_accel_ef().y == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(backend.get_accel_ef().z == Catch::Approx(-kGravity).margin(1e-4));
}

TEST_CASE("EkfCoreBackend: one tick with pre-set gyro_bias/accel_bias matches an independently-derived "
          "small-angle rotation, bias-corrected omega, and bias-corrected+rotated accel_ef",
          "[ekf_backend]") {
    // First-ever tick of a fresh EkfCoreBackend: EkfCore::tick()'s
    // internal IMU delay buffer isn't seeded yet, so this call mechanizes
    // a stationary/level NO-OP seed sample (zero delta_angle, gravity-
    // cancelling delta_velocity - see ekf_core.cpp's tick() pre-fill
    // comment), NOT the real sample this call passes in. With gyro_bias/
    // accel_bias pre-set to a known nonzero value, update_strapdown_
    // equations_ned()'s own real correction
    // (`gyro.delta_angle - state.gyro_bias * (gyro.delta_angle_dt / dt_ekf_avg)`,
    // verified directly against ekf_core.cpp) still applies to that seed
    // sample's zero delta_angle, so the mechanized rotation this one tick
    // is EXACTLY `-gyro_bias` (a small rotation vector, since dangle_dt ==
    // dt_ekf_avg for the seed) - not identity. This test accounts for
    // that explicitly (via the small-angle rotation-vector-as-Euler-angle
    // equivalence, valid to O(theta^3) for theta this small) rather than
    // assuming a trivially-identity DCM.
    ekf::EkfCoreBackend backend;

    const ekf::ftype gyro_bias_x = ekf::ftype(0.0010);
    const ekf::ftype gyro_bias_y = ekf::ftype(-0.0007);
    const ekf::ftype gyro_bias_z = ekf::ftype(0.0004);
    backend.ekf_core().state.gyro_bias = ekf::Vector3F(gyro_bias_x, gyro_bias_y, gyro_bias_z);

    // Kept well inside EkfCore::constrain_states()'s own real
    // accel_bias_lim (= acc_bias_lim(1.0) * dt_ekf_avg(0.01) = 0.01 m/s
    // here, ekf_core.cpp ~line 245) - values outside that range are
    // legitimately clamped by tick() itself before this test ever reads
    // state.accel_bias back, which would make this test's own
    // independently-computed "expected" value wrong (found the hard way:
    // an earlier version of this test used 0.02/-0.01/0.03, and 0.02/0.03
    // silently got clamped down to 0.01 by the real, correct
    // constrain_states() behavior, not a bug in EkfCoreBackend).
    const ekf::ftype accel_bias_x = ekf::ftype(0.006);
    const ekf::ftype accel_bias_y = ekf::ftype(-0.004);
    const ekf::ftype accel_bias_z = ekf::ftype(0.005);
    backend.ekf_core().state.accel_bias = ekf::Vector3F(accel_bias_x, accel_bias_y, accel_bias_z);

    const float dt = 0.01f;
    const math::Vector3f raw_gyro_rate(0.05f, -0.02f, 0.03f);   // rad/s, the sample's own `gyro` field
    const math::Vector3f raw_accel(0.4f, -0.3f, -9.75f);        // m/s^2, the sample's own `accel` field

    ahrs::GyroSample gyro;
    gyro.delta_angle = math::Vector3f(0.002f, -0.001f, 0.0015f); // irrelevant to this tick's mechanization (seed window)
    gyro.dangle_dt = dt;
    gyro.gyro = raw_gyro_rate;

    ahrs::AccelSample accel;
    accel.delta_velocity = math::Vector3f(0.4f, -0.3f, -9.75f) * dt; // also irrelevant this tick
    accel.delta_velocity_dt = dt;
    accel.accel = raw_accel;

    backend.update_full_cycle(gyro, accel, dt, ahrs::CompassSample{}, ahrs::GpsSample{}, false, false, false, 0.0f,
                               math::Vector3f{}, 0.0f, false, false, 0);

    // --- Independent reference #1: the attitude this tick's mechanization
    // must have produced. Built via Matrix3f::from_euler() - a completely
    // separate code path from QuaternionT::rotation_matrix() - treating
    // the (tiny) rotation vector's components directly as roll/pitch/yaw,
    // valid to O(theta^3) for a rotation this small (theta ~ 1e-3 rad).
    const float rot_x = static_cast<float>(-gyro_bias_x);
    const float rot_y = static_cast<float>(-gyro_bias_y);
    const float rot_z = static_cast<float>(-gyro_bias_z);
    math::Matrix3f expected_dcm;
    expected_dcm.from_euler(rot_x, rot_y, rot_z);

    REQUIRE(backend.get_dcm_matrix().a.x == Catch::Approx(expected_dcm.a.x).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().a.y == Catch::Approx(expected_dcm.a.y).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().a.z == Catch::Approx(expected_dcm.a.z).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().b.x == Catch::Approx(expected_dcm.b.x).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().b.y == Catch::Approx(expected_dcm.b.y).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().b.z == Catch::Approx(expected_dcm.b.z).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().c.x == Catch::Approx(expected_dcm.c.x).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().c.y == Catch::Approx(expected_dcm.c.y).margin(1e-6));
    REQUIRE(backend.get_dcm_matrix().c.z == Catch::Approx(expected_dcm.c.z).margin(1e-6));

    REQUIRE(backend.get_roll() == Catch::Approx(rot_x).margin(1e-6));
    REQUIRE(backend.get_pitch() == Catch::Approx(rot_y).margin(1e-6));
    REQUIRE(backend.get_yaw() == Catch::Approx(rot_z).margin(1e-6));

    // --- Independent reference #2: bias-corrected turn rate. state.
    // gyro_bias is untouched by mechanization (only quat/velocity/
    // position are written; fusion, which is the only thing that would
    // otherwise correct the bias states, is never called - see
    // ekf_backend.hpp's fusion-gating decision), so it is still exactly
    // what this test set it to.
    const float expected_omega_x = raw_gyro_rate.x - static_cast<float>(gyro_bias_x) / dt;
    const float expected_omega_y = raw_gyro_rate.y - static_cast<float>(gyro_bias_y) / dt;
    const float expected_omega_z = raw_gyro_rate.z - static_cast<float>(gyro_bias_z) / dt;
    REQUIRE(backend.get_omega().x == Catch::Approx(expected_omega_x).margin(1e-5));
    REQUIRE(backend.get_omega().y == Catch::Approx(expected_omega_y).margin(1e-5));
    REQUIRE(backend.get_omega().z == Catch::Approx(expected_omega_z).margin(1e-5));

    // --- Independent reference #3: bias-corrected body accel, rotated by
    // the (independently-built, above) expected_dcm - matching AhrsDcm's
    // own `accel_ef = dcm_matrix * sample.accel` convention (ahrs_dcm.hpp)
    // with EkfCore's own bias correction folded in first.
    const math::Vector3f corrected_body_accel(raw_accel.x - static_cast<float>(accel_bias_x) / dt,
                                               raw_accel.y - static_cast<float>(accel_bias_y) / dt,
                                               raw_accel.z - static_cast<float>(accel_bias_z) / dt);
    const math::Vector3f expected_accel_ef = expected_dcm * corrected_body_accel;
    REQUIRE(backend.get_accel_ef().x == Catch::Approx(expected_accel_ef.x).margin(1e-4));
    REQUIRE(backend.get_accel_ef().y == Catch::Approx(expected_accel_ef.y).margin(1e-4));
    REQUIRE(backend.get_accel_ef().z == Catch::Approx(expected_accel_ef.z).margin(1e-4));
}

TEST_CASE("EkfCoreBackend: many ticks of a constant yaw rate match the analytic same-axis rotation, "
          "accounting for EkfCore::tick()'s own disclosed IMU delay depth",
          "[ekf_backend]") {
    // Large-angle (as opposed to the previous test's small-angle) coverage
    // for get_dcm_matrix()/get_roll()/get_pitch()/get_yaw(): a constant
    // body-z rotation rate, zero bias, driven for many ticks - matching
    // ekf_core_test.cpp's own "constant angular rate" analytic-rotation
    // test precedent, but through the full update_full_cycle()/tick()
    // path (with its real IMU delay buffer) rather than a direct
    // update_strapdown_equations_ned() call.
    ekf::EkfCoreBackend backend;

    const float dt = 0.001f;
    const float omega_z = 2.0f; // rad/s
    const int steps = 100;

    ahrs::GyroSample gyro;
    gyro.delta_angle = math::Vector3f(0.0f, 0.0f, omega_z * dt);
    gyro.dangle_dt = dt;
    gyro.gyro = math::Vector3f(0.0f, 0.0f, omega_z);

    ahrs::AccelSample accel;
    accel.accel = math::Vector3f(0.0f, 0.0f, -kGravity); // level: purely vertical, unaffected by a yaw-only rotation
    accel.delta_velocity = accel.accel * dt;
    accel.delta_velocity_dt = dt;

    for (int i = 0; i < steps; ++i) {
        backend.update_full_cycle(gyro, accel, dt, ahrs::CompassSample{}, ahrs::GpsSample{}, false, false, false,
                                   0.0f, math::Vector3f{}, 0.0f, false, false, 0);
    }

    // EkfCore::tick() mechanizes the OLDEST buffered sample, delayed by
    // (kImuBufferCapacity - 1) real ticks behind the newest push (the
    // first kImuBufferCapacity-1 calls still read back one of the
    // zero-rotation seed slots, per tick()'s own pre-fill comment,
    // ekf_core.cpp) - a real, disclosed EkfCore::tick() behavior (CPP-071/
    // 073), not specific to this adapter. Since every pushed sample here
    // is IDENTICAL (constant rate), the exact number of real (non-seed)
    // rotation increments actually mechanized after `steps` calls is
    // exactly `steps - (kImuBufferCapacity - 1)`.
    static_assert(steps > static_cast<int>(ekf::EkfCore::kImuBufferCapacity) - 1,
                  "test must run long enough to clear the pre-fill window");
    const int real_rotation_ticks = steps - (static_cast<int>(ekf::EkfCore::kImuBufferCapacity) - 1);
    const float expected_yaw = omega_z * dt * static_cast<float>(real_rotation_ticks);

    REQUIRE(backend.get_roll() == Catch::Approx(0.0f).margin(1e-5));
    REQUIRE(backend.get_pitch() == Catch::Approx(0.0f).margin(1e-5));
    REQUIRE(backend.get_yaw() == Catch::Approx(expected_yaw).margin(1e-4));

    math::Matrix3f expected_dcm;
    expected_dcm.from_euler(0.0f, 0.0f, expected_yaw);
    REQUIRE(backend.get_dcm_matrix().a.x == Catch::Approx(expected_dcm.a.x).margin(1e-4));
    REQUIRE(backend.get_dcm_matrix().a.y == Catch::Approx(expected_dcm.a.y).margin(1e-4));
    REQUIRE(backend.get_dcm_matrix().b.x == Catch::Approx(expected_dcm.b.x).margin(1e-4));
    REQUIRE(backend.get_dcm_matrix().b.y == Catch::Approx(expected_dcm.b.y).margin(1e-4));
    REQUIRE(backend.get_dcm_matrix().c.z == Catch::Approx(expected_dcm.c.z).margin(1e-4));

    // No bias set anywhere in this test -> get_omega() is exactly the raw
    // commanded rate.
    REQUIRE(backend.get_omega().x == Catch::Approx(0.0f).margin(1e-9));
    REQUIRE(backend.get_omega().y == Catch::Approx(0.0f).margin(1e-9));
    REQUIRE(backend.get_omega().z == Catch::Approx(omega_z).margin(1e-6));

    // A yaw-only rotation can never tilt a purely-vertical vector - the
    // ticket's own required "stationary, level -> accel_ef.z close to
    // -GRAVITY_MSS" sanity check, this time demonstrated ACROSS a
    // non-trivial commanded rotation (not just the identity-attitude case
    // the previous TEST_CASE already covers) to prove get_accel_ef()'s
    // rotation step is genuinely axis-aware, not a coincidence of never
    // actually rotating.
    REQUIRE(backend.get_accel_ef().x == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(backend.get_accel_ef().y == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(backend.get_accel_ef().z == Catch::Approx(-kGravity).margin(1e-3));
}
