// Tests for fwcpp::sim::SimPlane (CPP-030: STANDARD-config ground-truth
// fixed-wing flight dynamics, ported from upstream SITL::Plane).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/sim/sim_plane.hpp>

using namespace fwcpp::sim;
using fwcpp::math::Vector3f;

namespace {

// Independently-transcribed reference formulas (not calling SimPlane's own
// liftCoeff/dragCoeff), matching ahrs_dcm_test.cpp's cross-check style:
// verify the class against a second, separately-written implementation of
// upstream's own math rather than reading the class's output back at itself.
double reference_lift_coeff(const Coefficients& c, float alpha) {
    const double alpha0 = c.alpha_stall;
    const double M = c.mcoeff;
    const double max_alpha_delta = 0.8;
    double a = alpha;
    if (a - alpha0 > max_alpha_delta) {
        a = alpha0 + max_alpha_delta;
    } else if (alpha0 - a > max_alpha_delta) {
        a = alpha0 - max_alpha_delta;
    }
    const double sigmoid = (1 + std::exp(-M * (a - alpha0)) + std::exp(M * (a + alpha0)))
                          / (1 + std::exp(-M * (a - alpha0))) / (1 + std::exp(M * (a + alpha0)));
    const double linear = (1.0 - sigmoid) * (c.c_lift_0 + c.c_lift_a * a);
    const double flatPlate = sigmoid * (2 * std::copysign(1.0, a) * std::pow(std::sin(a), 2) * std::cos(a));
    return linear + flatPlate;
}

double reference_drag_coeff(const Coefficients& c, float alpha) {
    const double AR = std::pow(static_cast<double>(c.b), 2) / c.s;
    return c.c_drag_p + std::pow(c.c_lift_0 + c.c_lift_a * alpha, 2) / (M_PI * c.oswald * AR);
}

} // namespace

TEST_CASE("liftCoeff matches an independently-computed reference across alpha", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    for (float alpha : {-0.6f, -0.2f, 0.0f, 0.1f, 0.3f, 0.6f, 1.2f, 2.5f}) {
        REQUIRE(plane.liftCoeff(alpha) == Catch::Approx(reference_lift_coeff(c, alpha)).margin(1e-4));
    }
}

TEST_CASE("liftCoeff rises through the linear regime then blends to the flat-plate stall model", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    // Small-alpha linear regime: lift increases with alpha, close to the
    // pure linear c_lift_0 + c_lift_a*alpha prediction (sigmoid ~ 0 far
    // below alpha_stall).
    const float lift_0 = plane.liftCoeff(0.0f);
    const float lift_small = plane.liftCoeff(0.2f);
    REQUIRE(lift_small > lift_0);
    REQUIRE(lift_0 == Catch::Approx(c.c_lift_0).margin(1e-3f));
    REQUIRE(lift_small == Catch::Approx(c.c_lift_0 + c.c_lift_a * 0.2f).margin(1e-2f));

    // Well past stall (alpha clamped to alpha_stall+0.8), the sigmoid-blended
    // flat-plate model must diverge substantially from naive linear
    // extrapolation - that divergence is the entire point of the model.
    const float alpha_deep_stall = 3.0f; // clamps to alpha_stall + 0.8
    const float lift_deep_stall = plane.liftCoeff(alpha_deep_stall);
    const float naive_linear = c.c_lift_0 + c.c_lift_a * alpha_deep_stall;
    REQUIRE(std::fabs(lift_deep_stall - naive_linear) > 1.0f);
}

TEST_CASE("dragCoeff matches an independently-computed reference and grows away from minimum-drag alpha", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;

    for (float alpha : {-0.5f, -0.08f, 0.0f, 0.3f, 0.8f}) {
        REQUIRE(plane.dragCoeff(alpha) == Catch::Approx(reference_drag_coeff(c, alpha)).margin(1e-5));
    }

    // Minimum induced drag is where c_lift_0 + c_lift_a*alpha == 0, i.e.
    // alpha ~ -c_lift_0/c_lift_a ~ -0.081 - moving away from it (either
    // direction) must increase drag.
    const float alpha_min_drag = -c.c_lift_0 / c.c_lift_a;
    const float drag_at_min = plane.dragCoeff(alpha_min_drag);
    REQUIRE(plane.dragCoeff(alpha_min_drag + 0.3f) > drag_at_min);
    REQUIRE(plane.dragCoeff(alpha_min_drag - 0.3f) > drag_at_min);
}

TEST_CASE("getForce returns exactly zero force when airspeed is exactly zero", "[sim_plane]") {
    SimPlane plane;
    const Vector3f force = plane.getForce(0.1f, 0.2f, -0.1f, 0.3f, 0.05f, 0.0f, Vector3f(0.1f, -0.2f, 0.05f), kSslAirDensity);
    REQUIRE(force == Vector3f(0.0f, 0.0f, 0.0f));
}

TEST_CASE("getTorque at zero airspeed reduces to exactly the CG-offset misalignment term", "[sim_plane]") {
    SimPlane plane;
    Coefficients c;
    const Vector3f force(2.0f, -1.0f, 0.5f);

    const Vector3f torque = plane.getTorque(0.1f, 0.2f, -0.1f, 0.5f, force,
                                             0.3f, 0.0f, 0.05f, Vector3f(0.1f, -0.2f, 0.05f), kSslAirDensity);

    const float expected_la = c.cg_offset.y * force.z - c.cg_offset.z * force.y;
    const float expected_ma = -c.cg_offset.x * force.z + c.cg_offset.z * force.x;
    const float expected_na = -c.cg_offset.y * force.x + c.cg_offset.x * force.y;

    REQUIRE(torque.x == Catch::Approx(expected_la));
    REQUIRE(torque.y == Catch::Approx(expected_ma));
    REQUIRE(torque.z == Catch::Approx(expected_na));
}

TEST_CASE("analytically-derived trim angle of attack roughly balances weight in getForce", "[sim_plane]") {
    // Physical sanity check (see ticket): at a plausible cruise airspeed,
    // solve the LINEAR-region lift equation for the alpha that makes
    // lift ~= weight, then confirm the actual (sigmoid-blended) model
    // agrees with that estimate to within a generous margin - this is
    // ap-sim's own oracle, so this checks internal physical plausibility,
    // not an external ground truth.
    SimPlane plane;
    Coefficients c;

    const float cruise_airspeed = 15.0f; // m/s, plausible for a ~1.9m-span light UAV
    const float weight = plane.mass * kGravityMss;
    const float qbar = 0.5f * kSslAirDensity * cruise_airspeed * cruise_airspeed;
    const float required_cl = weight / (qbar * c.s);
    const float alpha_trim = (required_cl - c.c_lift_0) / c.c_lift_a;

    const Vector3f force = plane.getForce(0.0f, 0.0f, 0.0f, alpha_trim, 0.0f, cruise_airspeed, Vector3f(0.0f, 0.0f, 0.0f), kSslAirDensity);

    // force.z is body-frame "down"; lift opposes it, so it should be
    // negative and close in magnitude to weight for this small alpha.
    REQUIRE(force.z == Catch::Approx(-weight).margin(weight * 0.2f));
}

TEST_CASE("update() from rest under constant throttle accelerates forward without crashing", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -500.0f); // 500m up, well clear of the ground
    plane.dcm.identity();

    const float dt = 0.005f;
    const int steps = 200; // 1 second
    for (int i = 0; i < steps; ++i) {
        plane.update(0.0f, 0.0f, 0.0f, plane.hover_throttle, dt);
    }

    REQUIRE_FALSE(plane.dcm.is_nan());
    REQUIRE_FALSE(std::isnan(plane.velocity_ef.x));
    REQUIRE_FALSE(std::isnan(plane.position.z));

    // Accelerated meaningfully from rest.
    REQUIRE(plane.velocity_ef.length() > 3.0f);

    // Direct regression pin for a real bug caught in review: update()
    // must actually recompute airspeed from velocity_air_bf each tick
    // (update_dynamics's Aircraft::update_eas_airspeed()-equivalent). If
    // that assignment is ever dropped again, airspeed stays frozen at its
    // zero-initialized default, is_zero(airspeed) in getForce/getTorque
    // is permanently true, and the aircraft silently generates zero
    // aerodynamic force for its entire life (pure free-fall-under-
    // thrust). The velocity/position bounds below are too loose to catch
    // that on their own within a 1-second window, so pin it directly.
    REQUIRE(plane.airspeed > 3.0f);

    // Didn't fall out of the sky: even completely unpowered/liftless free
    // fall only loses ~4.9m in 1s (0.5*g*t^2); this plane has both thrust
    // and lift building as airspeed increases, so a generous 10m bound
    // catches a genuinely broken integrator without being sensitive to
    // the (uncontrolled, undamped) pitch transient a neutral-surface plane
    // naturally has.
    REQUIRE(plane.position.z < -490.0f);
}

TEST_CASE("update_dynamics ground-contact clamp prevents sinking through the floor", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, 0.0f); // on the ground
    plane.velocity_ef = Vector3f(0.0f, 0.0f, 5.0f); // moving down into the ground
    plane.dcm.identity();

    const float dt = 0.01f;
    for (int i = 0; i < 3; ++i) {
        plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), dt);
        // Bounded: one step's worth of overshoot at most, never a runaway sink.
        REQUIRE(plane.position.z < 0.1f);
        REQUIRE(plane.position.z >= 0.0f);
    }

    // Downward velocity has been clamped away, not merely reduced.
    REQUIRE(plane.velocity_ef.z == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("update_dynamics clamps an extreme rot_accel to the +-2000 deg/s gyro-rate limit", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne, no ground interaction
    plane.dcm.identity();
    plane.gyro.zero();

    // Deliberately extreme rot_accel; a tiny dt keeps the resulting
    // dcm.rotate() step itself small even though the pre-clamp rate would
    // be enormous, isolating the clamp behavior from integrator blow-up.
    plane.update_dynamics(Vector3f(1.0e8f, 0.0f, 0.0f), 1.0e-6f);

    const float limit = fwcpp::math::radians(2000.0f);
    REQUIRE(plane.gyro.x == Catch::Approx(limit));
    REQUIRE(plane.gyro.y == Catch::Approx(0.0f).margin(1e-9f));
    REQUIRE(plane.gyro.z == Catch::Approx(0.0f).margin(1e-9f));
    REQUIRE_FALSE(plane.dcm.is_nan());
}

TEST_CASE("update_dynamics clamps an extreme body accel to +-64G", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f); // airborne, no ground clamp interaction
    plane.dcm.identity();
    plane.accel_body = Vector3f(1.0e8f, 0.0f, 0.0f); // deliberately extreme

    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.001f);

    // With dcm==identity and not on the ground, the final accelerometer-
    // equivalent accel_body (accel_earth + (0,0,-g)) equals the CLAMPED
    // input exactly - gravity is added then subtracted back out.
    const float limit = 64.0f * kGravityMss;
    REQUIRE(plane.accel_body.x == Catch::Approx(limit).margin(1e-2f));
    REQUIRE(plane.accel_body.y == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.accel_body.z == Catch::Approx(0.0f).margin(1e-2f));
}

TEST_CASE("repeated update_dynamics keeps the DCM orthonormal over many integration steps", "[sim_plane]") {
    SimPlane plane;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane.dcm.identity();

    const Vector3f rot_accel(0.05f, -0.03f, 0.02f);
    const float dt = 0.001f;
    for (int i = 0; i < 5000; ++i) {
        plane.update_dynamics(rot_accel, dt);
    }

    REQUIRE_FALSE(plane.dcm.is_nan());
    REQUIRE(plane.dcm.a.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(plane.dcm.b.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(plane.dcm.c.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE((plane.dcm.a * plane.dcm.b) == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(plane.dcm.det() == Catch::Approx(1.0f).margin(1e-3f));
}
