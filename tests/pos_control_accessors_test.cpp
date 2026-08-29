// CCP-027 slice 6: offsets, terrain, stopping accessors, update_estimates.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/poscontrol/pos_control_accessors.hpp>

using namespace fwcpp::poscontrol;
using namespace fwcpp::math;
using Catch::Matchers::WithinAbs;

TEST_CASE("update_estimates from injected AHRS NED", "[poscontrol][accessors][estimates]") {
    AhrsPosControlEstimateInputs ahrs{};
    ahrs.pos_ned_valid = true;
    ahrs.pos_ned_m = Vector3<postype_t>{10.0, -3.0, 5.0};
    ahrs.vel_ned_valid = true;
    ahrs.vel_ned_ms = Vector3f{1.0f, 2.0f, -0.5f};

    const NedEstimates est = update_estimates(ahrs);
    REQUIRE_THAT(static_cast<float>(est.pos_m.x), WithinAbs(10.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(est.pos_m.y), WithinAbs(-3.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(est.pos_m.z), WithinAbs(5.0f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.x, WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.y, WithinAbs(2.0f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.z, WithinAbs(-0.5f, 1e-4f));
}

TEST_CASE("update_estimates vertical fallback and high_vibes", "[poscontrol][accessors][estimates]") {
    NedEstimates prior{};
    prior.pos_m = Vector3<postype_t>{1.0, 2.0, 99.0};
    prior.vel_ms = Vector3f{0.1f, 0.2f, 0.3f};

    AhrsPosControlEstimateInputs ahrs{};
    ahrs.pos_d_valid = true;
    ahrs.pos_d_m = 7.5f;
    ahrs.high_vibes = true;
    ahrs.vert_rate_d_valid = true;
    ahrs.vert_rate_d_ms = -1.25f;

    const NedEstimates est = update_estimates(ahrs, prior);
    REQUIRE_THAT(static_cast<float>(est.pos_m.x), WithinAbs(1.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(est.pos_m.y), WithinAbs(2.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(est.pos_m.z), WithinAbs(7.5f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.x, WithinAbs(0.1f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.y, WithinAbs(0.2f, 1e-4f));
    REQUIRE_THAT(est.vel_ms.z, WithinAbs(-1.25f, 1e-4f));
}

TEST_CASE("init_terrain and init_pos_terrain_d_m", "[poscontrol][accessors][terrain]") {
    PosControlD d{};
    d.pos_desired_m = postype_t{20.0};
    DTerrain terrain = init_terrain();
    REQUIRE_THAT(static_cast<float>(terrain.pos_m), WithinAbs(0.0f, 1e-6f));

    init_pos_terrain_d_m(d, terrain, 12.0f);
    REQUIRE_THAT(static_cast<float>(terrain.pos_m), WithinAbs(12.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(d.pos_desired_m), WithinAbs(8.0f, 1e-4f));

    init_pos_terrain_u_cm(d, terrain, 500.0f);
    REQUIRE_THAT(static_cast<float>(terrain.pos_m), WithinAbs(-5.0f, 1e-4f));
}

TEST_CASE("get_stopping_point accessors", "[poscontrol][accessors][stopping]") {
    NedEstimates est{};
    est.pos_m = Vector3<postype_t>{0.0, 0.0, 10.0};
    est.vel_ms = Vector3f{2.0f, 0.0f, 1.0f};

    NeOffsets ne_off{};
    ne_off.pos_m = Vector2<postype_t>{1.0, 0.0};
    ne_off.vel_ms = Vector2f{0.5f, 0.0f};

    DOffsets d_off{};
    d_off.pos_m = postype_t{2.0};
    d_off.vel_ms = 0.25f;

    const NeLimits ne_limits = ne_set_max_speed_accel_m(5.0f, 2.0f, 10.0f, AttitudeCapability{});
    const auto stop_ne = get_stopping_point_ne_m(est, ne_off, 1.0f, ne_limits);
    REQUIRE(stop_ne.x > postype_t{-1.0});

    const DLimits d_limits = DLimits::defaults();
    const postype_t stop_d = get_stopping_point_d_m(est, d_off, 1.0f, d_limits);
    REQUIRE(stop_d > postype_t{7.0});
}

TEST_CASE("offset target setters and NED getters", "[poscontrol][accessors][offsets]") {
    NeOffsetState ne{};
    DOffsetState d{};
    set_posvelaccel_offset_target_ne_m(ne, Vector2<postype_t>{3.0, 4.0},
                                       Vector2f{0.5f, -0.5f}, Vector2f{0.1f, 0.2f}, 1000);
    set_posvelaccel_offset_target_d_m(d, 6.0f, -0.2f, 0.05f, 2000);
    ne.init(1000);
    d.init(2000);

    const auto pos = get_pos_offset_ned_m(ne, d);
    REQUIRE_THAT(static_cast<float>(pos.x), WithinAbs(3.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(pos.y), WithinAbs(4.0f, 1e-4f));
    REQUIRE_THAT(static_cast<float>(pos.z), WithinAbs(6.0f, 1e-4f));

    set_pos_offset_d_m(d, -4.0f);
    REQUIRE_THAT(get_pos_offset_u_m(d), WithinAbs(4.0f, 1e-4f));
    REQUIRE_THAT(get_vel_offset_d_ms(d), WithinAbs(-0.2f, 1e-4f));
    REQUIRE_THAT(get_accel_offset_d_mss(d), WithinAbs(0.05f, 1e-4f));
}
