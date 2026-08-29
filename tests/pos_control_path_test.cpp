// CCP-027 slice 7: 3D input_pos_NED_m path shaper.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/poscontrol/pos_control_path.hpp>

using namespace fwcpp::poscontrol;
using namespace fwcpp::math;
using Catch::Matchers::WithinAbs;

TEST_CASE("terrain_scaler_d_m margin bands", "[poscontrol][path][terrain]") {
    REQUIRE_THAT(terrain_scaler_d_m(10.0f, 10.0f, 0.0f, 0.0f, 0.0f), WithinAbs(1.0f, 1e-6f));
    const float mid = terrain_scaler_d_m(15.0f, 10.0f, 0.0f, 2.0f, 4.0f);
    REQUIRE_THAT(mid, WithinAbs(0.5f, 1e-4f));
    const float edge = terrain_scaler_d_m(18.0f, 10.0f, 0.0f, 2.0f, 4.0f);
    REQUIRE_THAT(edge, WithinAbs(0.01f, 1e-4f));
}

TEST_CASE("kinematic_limit diagonal and axis-aligned", "[poscontrol][path][kinematic]") {
    REQUIRE_THAT(kinematic_limit(1.0f, 0.0f, 5.0f, 3.0f, 2.0f), WithinAbs(5.0f, 1e-4f));
    REQUIRE_THAT(kinematic_limit(0.0f, 1.0f, 5.0f, 3.0f, 2.0f), WithinAbs(2.0f, 1e-4f));
    const Vector3f dir{3.0f, 4.0f, 0.0f};
    REQUIRE_THAT(kinematic_limit(dir, 10.0f, 5.0f, 5.0f), WithinAbs(10.0f, 1e-4f));
}

TEST_CASE("input_pos_ned_m drives NE acceleration demand", "[poscontrol][path][input]") {
    PosControlNe ne{};
    PosControlD d{};
    DTerrain terrain{};
    ne.pos_desired_m = Vector2<postype_t>{0.0, 0.0};
    d.pos_desired_m = postype_t{0.0};

    Vector3<postype_t> pos{50.0, 0.0, 0.0};
    InputPosNedPathContext ctx{};
    ctx.dt = 0.1f;
    ctx.vel_max_ne_ms = 5.0f;
    ctx.ne_limits = ne_set_max_speed_accel_m(5.0f, 2.0f, 10.0f, AttitudeCapability{});
    ctx.d_limits = DLimits::defaults();

    input_pos_ned_m(pos, 0.0f, 0.0f, ne, d, terrain, ctx);
    REQUIRE(ne.accel_desired_mss.length() > 0.1f);
    REQUIRE_THAT(static_cast<float>(terrain.pos_target_m), WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("input_pos_ned_m terrain scaler slows NE motion", "[poscontrol][path][input]") {
    PosControlNe ne_fast{};
    PosControlD d_fast{};
    DTerrain terrain_fast{};
    PosControlNe ne_slow{};
    PosControlD d_slow{};
    DTerrain terrain_slow{};

    Vector3<postype_t> pos_fast{40.0, 0.0, 5.0};
    Vector3<postype_t> pos_slow{40.0, 0.0, 5.0};

    InputPosNedPathContext ctx{};
    ctx.dt = 0.1f;
    ctx.pos_estimate_d_m = 12.0f;
    ctx.pos_target_d_m = 10.0f;
    ctx.vel_max_ne_ms = 8.0f;
    ctx.ne_limits = ne_set_max_speed_accel_m(8.0f, 3.0f, 12.0f, AttitudeCapability{});
    ctx.d_limits = DLimits::defaults();

    input_pos_ned_m(pos_fast, 0.0f, 0.0f, ne_fast, d_fast, terrain_fast, ctx);

    ctx.pos_estimate_d_m = 18.0f;
    input_pos_ned_m(pos_slow, 2.0f, 4.0f, ne_slow, d_slow, terrain_slow, ctx);

    REQUIRE(ne_slow.accel_desired_mss.length() < ne_fast.accel_desired_mss.length());
}
