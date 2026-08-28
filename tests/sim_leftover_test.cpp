// CPP-030 leftover closer: SimPlane ground_behavior + FW airframe mix
// stubs + leftover-complete catalog.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/sim/sim_leftover.hpp>
#include <fwcpp/sim/sim_plane.hpp>

using namespace fwcpp::sim;
using fwcpp::math::Vector3f;

TEST_CASE("leftover catalog: Remaining is empty; tailsitter and GCS are OutOfScope",
          "[sim_plane][leftover][catalog]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(on_main_count() >= 5);
    REQUIRE(this_slice_count() >= 7);
    REQUIRE(out_of_scope_count() == 2);

    REQUIRE(completeness_has("skywalker_2013 aero + rigid-body integrator", PortStatus::kOnMain));
    REQUIRE(completeness_has("Wind modeling (CPP-051)", PortStatus::kOnMain));
    REQUIRE(completeness_has("ground_behavior NONE / NO_MOVEMENT / FWD_ONLY", PortStatus::kThisSlice));
    REQUIRE(completeness_has("elevons mix", PortStatus::kThisSlice));
    REQUIRE(completeness_has("vtail mix", PortStatus::kThisSlice));
    REQUIRE(completeness_has("tailsitter (airframe + GROUND_BEHAVIOR_TAILSITTER)", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("GCS / MAVLink / fill_fdm / hit-ground text", PortStatus::kOutOfScope));
    REQUIRE(sim_completeness_size()
            == on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
}

TEST_CASE("mix_surfaces default FrameConfig is identity", "[sim_plane][leftover][mix]") {
    SimPlane plane;
    const SurfaceDeflections out = plane.mix_surfaces(0.4f, -0.2f, 0.1f, 0.7f);
    REQUIRE(out.aileron == Catch::Approx(0.4f));
    REQUIRE(out.elevator == Catch::Approx(-0.2f));
    REQUIRE(out.rudder == Catch::Approx(0.1f));
    REQUIRE(out.throttle == Catch::Approx(0.7f));
}

TEST_CASE("elevons mix matches independently-transcribed upstream formula", "[sim_plane][leftover][mix]") {
    SimPlane plane;
    plane.frame_config.mix = AirframeMix::kElevons;
    const float ch1 = 0.4f;
    const float ch2 = -0.2f;
    const SurfaceDeflections out = plane.mix_surfaces(ch1, ch2, 0.9f, 0.5f);
    REQUIRE(out.aileron == Catch::Approx((ch2 - ch1) / 2.0f));
    REQUIRE(out.elevator == Catch::Approx(-(ch2 + ch1) / 2.0f));
    REQUIRE(out.rudder == Catch::Approx(0.0f));
    REQUIRE(out.throttle == Catch::Approx(0.5f));
}

TEST_CASE("vtail mix matches independently-transcribed upstream formula", "[sim_plane][leftover][mix]") {
    SimPlane plane;
    plane.frame_config.mix = AirframeMix::kVtail;
    const float ch1 = 0.3f;
    const float ch2 = -0.5f;
    const SurfaceDeflections out = plane.mix_surfaces(0.2f, ch1, ch2, 0.6f);
    REQUIRE(out.aileron == Catch::Approx(0.2f));
    REQUIRE(out.elevator == Catch::Approx((ch2 - ch1) / 2.0f));
    REQUIRE(out.rudder == Catch::Approx((ch2 + ch1) / 2.0f));
    REQUIRE(out.throttle == Catch::Approx(0.6f));
}

TEST_CASE("reverse_elevator_rudder negates elevator and rudder before mix", "[sim_plane][leftover][mix]") {
    SimPlane plane;
    plane.frame_config.reverse_elevator_rudder = true;
    const SurfaceDeflections out = plane.mix_surfaces(0.1f, 0.4f, -0.2f, 0.3f);
    REQUIRE(out.aileron == Catch::Approx(0.1f));
    REQUIRE(out.elevator == Catch::Approx(-0.4f));
    REQUIRE(out.rudder == Catch::Approx(0.2f));
}

TEST_CASE("dspoilers leftover mix averages paired spoilers", "[sim_plane][leftover][mix]") {
    DspoilerInputs in;
    in.dspoiler1_left = 0.2f;
    in.dspoiler2_left = 0.4f;
    in.dspoiler1_right = -0.2f;
    in.dspoiler2_right = 0.0f;
    const SurfaceDeflections out = SimPlane::mix_dspoilers(in);
    const float elevon_left = (0.2f + 0.4f) / 2.0f;
    const float elevon_right = (-0.2f + 0.0f) / 2.0f;
    REQUIRE(out.aileron == Catch::Approx((elevon_right - elevon_left) / 2.0f));
    REQUIRE(out.elevator == Catch::Approx((elevon_left + elevon_right) / 2.0f));
}

TEST_CASE("redundant leftover mix averages paired channels", "[sim_plane][leftover][mix]") {
    RedundantInputs in;
    in.aileron_left = 0.2f;
    in.aileron_right = 0.4f;
    in.elevator_left = -0.1f;
    in.elevator_right = 0.1f;
    in.rudder_top = 0.3f;
    in.rudder_bottom = 0.1f;
    const SurfaceDeflections out = SimPlane::mix_redundant(in);
    REQUIRE(out.aileron == Catch::Approx(0.3f));
    REQUIRE(out.elevator == Catch::Approx(0.0f));
    REQUIRE(out.rudder == Catch::Approx(0.2f));
}

TEST_CASE("kNone ground_behavior preserves existing floor clamp only", "[sim_plane][leftover][ground]") {
    SimPlane plane;
    REQUIRE(plane.ground_behavior == GroundBehavior::kNone);
    plane.position = Vector3f(0.0f, 0.0f, 0.0f);
    plane.velocity_ef = Vector3f(3.0f, 1.0f, 5.0f);
    plane.gyro = Vector3f(0.1f, 0.0f, 0.0f);
    plane.dcm.identity();
    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);
    REQUIRE(plane.velocity_ef.z == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.velocity_ef.x != Catch::Approx(0.0f).margin(1e-3f));
    REQUIRE(plane.gyro.x != Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("kNoMovement zeros roll/pitch, xy velocity, and gyro on the ground",
          "[sim_plane][leftover][ground]") {
    SimPlane plane;
    plane.ground_behavior = GroundBehavior::kNoMovement;
    plane.position = Vector3f(0.0f, 0.0f, 0.0f);
    plane.velocity_ef = Vector3f(4.0f, -2.0f, 3.0f);
    plane.gyro = Vector3f(0.2f, -0.1f, 0.3f);
    plane.dcm.from_euler(0.3f, -0.2f, 0.5f);
    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);

    float r = 0.0f;
    float p = 0.0f;
    float y = 0.0f;
    plane.dcm.to_euler(&r, &p, &y);
    REQUIRE(r == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(p == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(y == Catch::Approx(0.5f).margin(0.05f));
    REQUIRE(plane.velocity_ef.x == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.velocity_ef.y == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.velocity_ef.z == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.gyro.x == Catch::Approx(0.0f));
    REQUIRE(plane.gyro.y == Catch::Approx(0.0f));
    REQUIRE(plane.gyro.z == Catch::Approx(0.0f));
}

TEST_CASE("kFwdOnly drops reverse and lateral body velocity on the ground",
          "[sim_plane][leftover][ground]") {
    SimPlane plane;
    plane.ground_behavior = GroundBehavior::kFwdOnly;
    plane.position = Vector3f(0.0f, 0.0f, 0.0f);
    plane.velocity_ef = Vector3f(-3.0f, 2.0f, 1.0f);
    plane.gyro = Vector3f(0.1f, 0.1f, 0.1f);
    plane.dcm.identity();
    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);

    const Vector3f v_bf = plane.dcm.transposed() * plane.velocity_ef;
    REQUIRE(v_bf.x >= -1e-5f);
    REQUIRE(v_bf.y == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(plane.velocity_ef.z == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(plane.gyro.x == Catch::Approx(0.0f));
    REQUIRE(plane.gyro.y == Catch::Approx(0.0f));
    REQUIRE(plane.gyro.z == Catch::Approx(0.0f));
}

TEST_CASE("kFwdOnly does not constrain an airborne aircraft", "[sim_plane][leftover][ground]") {
    SimPlane plane;
    plane.ground_behavior = GroundBehavior::kFwdOnly;
    plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    plane.velocity_ef = Vector3f(-3.0f, 2.0f, 0.0f);
    plane.gyro = Vector3f(0.1f, 0.0f, 0.0f);
    plane.dcm.identity();
    plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);
    REQUIRE(plane.velocity_ef.x == Catch::Approx(-3.0f).margin(0.2f));
    REQUIRE(plane.gyro.x == Catch::Approx(0.1f).margin(1e-4f));
}

TEST_CASE("kTailsitter leftover is a no-op (out of scope)", "[sim_plane][leftover][ground]") {
    SimPlane none_plane;
    SimPlane tail_plane;
    tail_plane.ground_behavior = GroundBehavior::kTailsitter;
    none_plane.position = Vector3f(0.0f, 0.0f, 0.0f);
    tail_plane.position = Vector3f(0.0f, 0.0f, 0.0f);
    none_plane.velocity_ef = Vector3f(2.0f, 1.0f, 1.0f);
    tail_plane.velocity_ef = Vector3f(2.0f, 1.0f, 1.0f);
    none_plane.dcm.identity();
    tail_plane.dcm.identity();
    none_plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);
    tail_plane.update_dynamics(Vector3f(0.0f, 0.0f, 0.0f), 0.01f);
    REQUIRE(tail_plane.velocity_ef.x == Catch::Approx(none_plane.velocity_ef.x).margin(1e-5f));
    REQUIRE(tail_plane.velocity_ef.y == Catch::Approx(none_plane.velocity_ef.y).margin(1e-5f));
}

TEST_CASE("default mix leaves update() STANDARD-config force path unchanged",
          "[sim_plane][leftover][regression]") {
    SimPlane std_plane;
    SimPlane leftover_plane;
    leftover_plane.frame_config.mix = AirframeMix::kStandard;
    std_plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    leftover_plane.position = Vector3f(0.0f, 0.0f, -100.0f);
    std_plane.velocity_ef = Vector3f(18.0f, 0.0f, 0.0f);
    leftover_plane.velocity_ef = Vector3f(18.0f, 0.0f, 0.0f);
    leftover_plane.velocity_air_bf = leftover_plane.velocity_ef;
    leftover_plane.airspeed = leftover_plane.velocity_ef.length();
    std_plane.velocity_air_bf = std_plane.velocity_ef;
    std_plane.airspeed = std_plane.velocity_ef.length();

    const float dt = 0.01f;
    for (int i = 0; i < 20; ++i) {
        std_plane.update(0.1f, -0.05f, 0.02f, std_plane.hover_throttle, dt);
        leftover_plane.update(0.1f, -0.05f, 0.02f, leftover_plane.hover_throttle, dt);
    }
    REQUIRE(std_plane.position.x == Catch::Approx(leftover_plane.position.x).margin(1e-4f));
    REQUIRE(std_plane.position.z == Catch::Approx(leftover_plane.position.z).margin(1e-4f));
}
