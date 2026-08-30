// CCP-044/045: copter_sitl_run leftover mission on real Frame/Motor plant.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/hal_sitl/copter_sitl_run_leftover.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

using fwcpp::copter::LeftoverCopter;
using fwcpp::hal_sitl::SitlCopterHarness;
using fwcpp::hal_sitl::copter_sitl_run::LeftoverMission;
using fwcpp::hal_sitl::copter_sitl_run::MissionPhase;
using fwcpp::hal_sitl::copter_sitl_run::PortStatus;
using fwcpp::hal_sitl::copter_sitl_run::completeness_has;
using fwcpp::hal_sitl::copter_sitl_run::completeness_size;
using fwcpp::hal_sitl::copter_sitl_run::leftover_copter_sitl_step;
using fwcpp::hal_sitl::copter_sitl_run::leftover_mission_begin_takeoff;
using fwcpp::hal_sitl::copter_sitl_run::on_main_count;
using fwcpp::hal_sitl::copter_sitl_run::out_of_scope_count;
using fwcpp::hal_sitl::copter_sitl_run::remaining_count;
using fwcpp::hal_sitl::copter_sitl_run::this_slice_count;
using fwcpp::sim::SimMulticopter;

TEST_CASE("zero command stays on ground", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_copter_sitl_step(harness, mission, 0.0025f);
    REQUIRE(sim.on_ground());
    REQUIRE((-sim.position.z) == Catch::Approx(0.0f).margin(0.01f));
}

TEST_CASE("climb command via leftover PWM leaves the ground", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    copter.motors_armed = true;
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_mission_begin_takeoff(mission);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 1200; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
    }
    REQUIRE((-sim.position.z) > 2.0f);
    REQUIRE_FALSE(sim.on_ground());
}

TEST_CASE("hoverThrOut holds altitude on the real plant", "[copter][sitl][ccp-045]") {
    SimMulticopter sim{};
    sim.position.z = -10.0f;
    sim.velocity_ef = {};
    LeftoverCopter copter{};
    copter.motors_armed = true;
    SitlCopterHarness harness(copter, sim);
    const float hover = sim.hover_command();
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 400; ++i) {
        fwcpp::hal_sitl::copter_sitl_run::leftover_apply_collective(copter, sim, hover);
        harness.step(kDt);
    }
    REQUIRE((-sim.position.z) == Catch::Approx(10.0f).margin(1.5f));
}

TEST_CASE("leftover copter sitl mission arm takeoff hold land",
          "[copter][sitl][ccp-044][ccp-045]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    SitlCopterHarness harness(copter, sim);
    LeftoverMission mission{};
    leftover_mission_begin_takeoff(mission);

    constexpr float kDt = 0.0025f;
    float max_alt_m = 0.0f;
    bool saw_hold = false;
    const int max_ticks = 20 * 400;
    for (int i = 0; i < max_ticks; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
        const float alt = -sim.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        if (mission.phase == MissionPhase::kHold) {
            saw_hold = true;
        }
        if (mission.phase == MissionPhase::kLanded) {
            break;
        }
    }

    REQUIRE(saw_hold);
    REQUIRE(max_alt_m >= 9.0f);
    REQUIRE(mission.phase == MissionPhase::kLanded);
    REQUIRE(copter.land_complete);
    REQUIRE_FALSE(copter.motors_armed);
    REQUIRE(sim.on_ground());
    REQUIRE(harness.tick_count() > 0);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.baro_injected);
}

TEST_CASE("copter_sitl_run leftover catalog remaining_count",
          "[copter][sitl][ccp-044][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 7);
    REQUIRE(on_main_count() == 3);
    REQUIRE(out_of_scope_count() == 4);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover_mission_advance", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_hold_command", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_apply_collective", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_sitl_step", PortStatus::kThisSlice));
    REQUIRE(completeness_has("copter_sitl_run arm/takeoff/hold/land", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_Multicopter Frame/Motor mixing", PortStatus::kOnMain));
}
