// CPP-088 slice 2: leftover catalog for remaining AP_HAL SITL surfaces.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/leftover.hpp>

using fwcpp::hal::PortStatus;
using fwcpp::hal::completeness_has;
using fwcpp::hal::hal_completeness_size;
using fwcpp::hal::on_main_count;
using fwcpp::hal::out_of_scope_count;
using fwcpp::hal::remaining_count;
using fwcpp::hal::this_slice_count;

TEST_CASE("hal leftover catalog lists this slice vs remaining", "[hal][leftover][catalog]") {
    REQUIRE(on_main_count() == 4);
    REQUIRE(this_slice_count() == 3);
    REQUIRE(remaining_count() > 0);
    REQUIRE(remaining_count() == 3);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(hal_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("GPIO", PortStatus::kOnMain));
    REQUIRE(completeness_has("Semaphore", PortStatus::kOnMain));
    REQUIRE(completeness_has("BinarySemaphore", PortStatus::kOnMain));
    REQUIRE(completeness_has("completeness catalog", PortStatus::kOnMain));
    REQUIRE(completeness_has("I2CDevice", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SPIDevice", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Device register access", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Util", PortStatus::kRemaining));
    REQUIRE(completeness_has("WSPI", PortStatus::kRemaining));
    REQUIRE(completeness_has("CAN", PortStatus::kRemaining));
}
