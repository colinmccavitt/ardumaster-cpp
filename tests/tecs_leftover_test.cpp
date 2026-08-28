// CPP-029 leftover closer: leftover TECS surfaces + leftover-complete catalog.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tecs/tecs.hpp>
#include <fwcpp/tecs/tecs_leftover.hpp>

using namespace fwcpp::tecs;

namespace {

TecsInputs default_inputs(std::uint64_t now_us, std::uint32_t now_ms) {
    TecsInputs in;
    in.rotation_body_to_ned.identity();
    in.eas2tas = 1.0f;
    in.using_airspeed_sensor = true;
    in.airspeed_eas_valid = true;
    in.airspeed_eas = 12.0f;
    in.velocity_ned_valid = true;
    in.velocity_down_ms = 0.0f;
    in.now_us = now_us;
    in.now_ms = now_ms;
    return in;
}

void settle(Tecs& tecs, TecsInputs& in, std::uint64_t& now_us, std::uint32_t& now_ms, float height_m,
            const TecsLandingInputs& landing, int ticks) {
    for (int i = 0; i < ticks; ++i) {
        now_us += 20000;
        now_ms += 20;
        in.now_us = now_us;
        in.now_ms = now_ms;
        in.relative_position_d_home_m = -height_m;
        tecs.update_50hz(in);
        tecs.update_pitch_throttle(static_cast<std::int32_t>(height_m * 100.0f), 1200, height_m, 1.0f, in, landing);
    }
}

}  // namespace

TEST_CASE("leftover catalog: Remaining is empty; VTOL and GCS are OutOfScope", "[tecs][leftover][catalog]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(on_main_count() >= 4);
    REQUIRE(this_slice_count() >= 6);
    REQUIRE(out_of_scope_count() == 2);

    REQUIRE(completeness_has("NORMAL-stage energy control law", PortStatus::kOnMain));
    REQUIRE(completeness_has("TECS flare height-rate blend", PortStatus::kOnMain));
    REQUIRE(completeness_has("get_land_sinkrate", PortStatus::kOnMain));
    REQUIRE(completeness_has("update_throttle_without_airspeed", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_land_airspeed", PortStatus::kThisSlice));
    REQUIRE(completeness_has("set_path_proportion", PortStatus::kThisSlice));
    REQUIRE(completeness_has("VTOL flight-stage branches", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("GCS send_TECS_status / HAL_LOGGING_ENABLED", PortStatus::kOutOfScope));
    REQUIRE(tecs_completeness_size() == on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
}

TEST_CASE("leftover accessors: get_land_airspeed default -1; set_path_proportion constrains [0,1]",
          "[tecs][leftover][accessors]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    REQUIRE(tecs.get_land_airspeed() == Catch::Approx(-1.0f));
    REQUIRE(tecs.leftover_path_proportion() == Catch::Approx(0.0f));
    REQUIRE(tecs.leftover_is_doing_auto_land() == false);
    REQUIRE(tecs.leftover_flight_stage() == TecsFlightStage::kNormal);
    REQUIRE(tecs.leftover_reached_speed_takeoff() == false);

    tecs.set_path_proportion(0.4f);
    REQUIRE(tecs.leftover_path_proportion() == Catch::Approx(0.4f));
    tecs.set_path_proportion(-0.5f);
    REQUIRE(tecs.leftover_path_proportion() == Catch::Approx(0.0f));
    tecs.set_path_proportion(1.7f);
    REQUIRE(tecs.leftover_path_proportion() == Catch::Approx(1.0f));

    gains.land_airspeed = 11.0f;
    Tecs tecs_set(gains, aparm);
    REQUIRE(tecs_set.get_land_airspeed() == Catch::Approx(11.0f));
}

TEST_CASE("leftover FlightStage / is_doing_auto_land surfaces store without changing NORMAL throttle law",
          "[tecs][leftover][stubs]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs_normal(gains, aparm);
    Tecs tecs_leftover(gains, aparm);

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -50.0f;
    tecs_normal.update_50hz(in);
    tecs_leftover.update_50hz(in);

    TecsLandingInputs leftover;
    leftover.is_doing_auto_land = true;
    leftover.leftover_flight_stage = TecsFlightStage::kTakeoff;

    std::uint64_t now_us_l = now_us;
    std::uint32_t now_ms_l = now_ms;
    TecsInputs in_l = in;
    settle(tecs_normal, in, now_us, now_ms, 50.0f, {}, 80);
    settle(tecs_leftover, in_l, now_us_l, now_ms_l, 50.0f, leftover, 80);

    REQUIRE(tecs_leftover.leftover_is_doing_auto_land() == true);
    REQUIRE(tecs_leftover.leftover_flight_stage() == TecsFlightStage::kTakeoff);
    REQUIRE(tecs_leftover.leftover_reached_speed_takeoff() == false);
    REQUIRE(tecs_normal.get_throttle_demand() == tecs_leftover.get_throttle_demand());
    REQUIRE(tecs_normal.get_pitch_demand() == tecs_leftover.get_pitch_demand());
}

TEST_CASE("update_throttle_without_airspeed: level flight demands near cruise; nose-up demands more",
          "[tecs][leftover][no-airspeed]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;

    auto run = [&](float pitch_rad) {
        Tecs tecs(gains, aparm);
        std::uint64_t now_us = 5'000'000;
        std::uint32_t now_ms = 1000;
        TecsInputs in = default_inputs(now_us, now_ms);
        in.using_airspeed_sensor = false;
        in.airspeed_eas_valid = false;
        in.pitch_rad = pitch_rad;
        in.relative_position_d_home_m = -50.0f;
        tecs.update_50hz(in);
        TecsLandingInputs landing;
        settle(tecs, in, now_us, now_ms, 50.0f, landing, 80);
        REQUIRE(tecs.use_airspeed() == false);
        REQUIRE(tecs.using_airspeed_for_throttle() == false);
        return tecs.get_throttle_demand();
    };

    const float thr_level = run(0.0f);
    const float thr_up = run(0.15f);
    const float thr_down = run(-0.15f);

    REQUIRE(thr_level == Catch::Approx(aparm.throttle_cruise).margin(8.0f));
    REQUIRE(thr_up > thr_level);
    REQUIRE(thr_down < thr_level);
}

TEST_CASE("update_throttle_without_airspeed: gliding zeros throttle; LAND_THR overrides cruise when leftover landing",
          "[tecs][leftover][no-airspeed]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    aparm.throttle_max = 0.0f;  // forces is_gliding

    Tecs glide(gains, aparm);
    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.using_airspeed_sensor = false;
    in.airspeed_eas_valid = false;
    in.relative_position_d_home_m = -50.0f;
    glide.update_50hz(in);
    settle(glide, in, now_us, now_ms, 50.0f, {}, 40);
    REQUIRE(glide.get_throttle_demand() == Catch::Approx(0.0f));

    Tecs::Gains land_gains;
    land_gains.land_throttle = 30.0f;
    Tecs::FixedWingParams land_aparm;
    Tecs land(land_gains, land_aparm);
    now_us = 5'000'000;
    now_ms = 1000;
    in = default_inputs(now_us, now_ms);
    in.using_airspeed_sensor = false;
    in.airspeed_eas_valid = false;
    in.relative_position_d_home_m = -50.0f;
    land.update_50hz(in);
    TecsLandingInputs landing;
    landing.is_doing_auto_land = true;
    settle(land, in, now_us, now_ms, 50.0f, landing, 80);
    REQUIRE(land.leftover_is_doing_auto_land() == true);
    REQUIRE(land.get_throttle_demand() == Catch::Approx(30.0f).margin(8.0f));
}

TEST_CASE("update_pitch_throttle: airspeed-on path still uses with-airspeed (leftover regression)",
          "[tecs][leftover][regression]") {
    Tecs::Gains gains;
    Tecs::FixedWingParams aparm;
    Tecs tecs(gains, aparm);

    std::uint64_t now_us = 5'000'000;
    std::uint32_t now_ms = 1000;
    TecsInputs in = default_inputs(now_us, now_ms);
    in.relative_position_d_home_m = -50.0f;
    tecs.update_50hz(in);
    settle(tecs, in, now_us, now_ms, 50.0f, {}, 40);

    REQUIRE(tecs.use_airspeed() == true);
    REQUIRE(tecs.using_airspeed_for_throttle() == true);
}
