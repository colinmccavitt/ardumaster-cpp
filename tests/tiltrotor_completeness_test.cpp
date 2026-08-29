#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tiltrotor/tiltrotor_completeness.hpp>

using fwcpp::tiltrotor::PortStatus;
using fwcpp::tiltrotor::completeness_has;
using fwcpp::tiltrotor::on_main_count;
using fwcpp::tiltrotor::remaining_count;
using fwcpp::tiltrotor::tiltrotor_completeness_size;
using fwcpp::tiltrotor::this_slice_count;

TEST_CASE("tiltrotor catalog", "[tiltrotor][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 27);
    REQUIRE(remaining_count() == 3);
    REQUIRE(tiltrotor_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 3);
    REQUIRE(completeness_has("tilt_max_change", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tiltrotor_Transition yaw/view/vfwd", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tiltrotor::continuous_update", PortStatus::kThisSlice));
    REQUIRE(completeness_has("tilt_over_max_angle", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tiltrotor::vectoring", PortStatus::kThisSlice));
    REQUIRE(completeness_has("tilt_compensate_angle", PortStatus::kThisSlice));
    REQUIRE(completeness_has("tilt_compensate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("setup thrust_type/motor scan", PortStatus::kThisSlice));
    REQUIRE(completeness_has("setup SRV tilt servo ranges", PortStatus::kThisSlice));
    REQUIRE(completeness_has("thrust compensation callback", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tiltrotor::bicopter_output", PortStatus::kRemaining));
    REQUIRE(completeness_has("get_forward_throttle", PortStatus::kRemaining));
    REQUIRE(completeness_has("write_log TRTL", PortStatus::kRemaining));
}
