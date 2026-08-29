// CPP-088 slice 3: SITL Util soft_armed / safety_switch / system_id.

#include <cstring>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/leftover.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/hal/util.hpp>

using fwcpp::hal::PortStatus;
using fwcpp::hal::RcOutput;
using fwcpp::hal::Util;
using fwcpp::hal::completeness_has;
using fwcpp::hal::kSitlAvailableMemoryBytes;
using fwcpp::hal::remaining_count;

TEST_CASE("soft_armed edge updates last_armed_change and persistent_data.armed",
          "[hal][util][soft_armed]") {
    Util util;
    REQUIRE_FALSE(util.get_soft_armed());
    REQUIRE(util.get_last_armed_change() == 0);
    REQUIRE_FALSE(util.persistent_data.armed);

    util.set_soft_armed(true, 1000);
    REQUIRE(util.get_soft_armed());
    REQUIRE(util.get_last_armed_change() == 1000);
    REQUIRE(util.persistent_data.armed);

    util.set_soft_armed(true, 2000);
    REQUIRE(util.get_soft_armed());
    REQUIRE(util.get_last_armed_change() == 1000);
    REQUIRE(util.persistent_data.armed);

    util.set_soft_armed(false, 3000);
    REQUIRE_FALSE(util.get_soft_armed());
    REQUIRE(util.get_last_armed_change() == 3000);
    REQUIRE_FALSE(util.persistent_data.armed);
}

TEST_CASE("persistent_data.armed is skipped when was_watchdog_reset",
          "[hal][util][watchdog]") {
    Util util;
    util.persistent_data.armed = true;
    util.persistent_data.safety_state = Util::SafetyState::kArmed;
    util.set_was_watchdog_reset(true);

    REQUIRE(util.was_watchdog_reset());
    REQUIRE(util.was_watchdog_armed());
    REQUIRE(util.was_watchdog_safety_off());
    REQUIRE(util.last_persistent_data.armed);

    util.set_soft_armed(true, 100);
    REQUIRE(util.get_soft_armed());
    REQUIRE(util.persistent_data.armed);

    util.set_soft_armed(false, 200);
    REQUIRE_FALSE(util.get_soft_armed());
    REQUIRE(util.get_last_armed_change() == 200);
    REQUIRE(util.persistent_data.armed);
    REQUIRE(util.was_watchdog_armed());
}

TEST_CASE("safety_switch_state follows RcOutput force_safety", "[hal][util][safety]") {
    RcOutput rcout;
    Util util{rcout};

    REQUIRE(util.safety_switch_state() == Util::SafetyState::kDisarmed);
    REQUIRE(rcout.safety_state() == fwcpp::hal::SafetyState::kDisarmed);

    rcout.force_safety_off();
    REQUIRE(util.safety_switch_state() == Util::SafetyState::kArmed);
    REQUIRE(rcout.safety_state() == fwcpp::hal::SafetyState::kArmed);

    rcout.force_safety_on();
    REQUIRE(util.safety_switch_state() == Util::SafetyState::kDisarmed);

    Util unbound;
    REQUIRE(unbound.safety_switch_state() == Util::SafetyState::kDisarmed);
    unbound.set_safety_switch_state(Util::SafetyState::kNone);
    REQUIRE(unbound.safety_switch_state() == Util::SafetyState::kNone);
    unbound.set_safety_switch_state(Util::SafetyState::kArmed);
    REQUIRE(unbound.safety_switch_state() == Util::SafetyState::kArmed);
}

TEST_CASE("system_id round-trips from an injected buffer", "[hal][util][system_id]") {
    Util util;
    const char raw[] = "sitl-injected-id";
    util.set_system_id(reinterpret_cast<const std::uint8_t*>(raw),
                       static_cast<std::uint8_t>(std::strlen(raw)));

    char buf[50] = {};
    REQUIRE(util.get_system_id(buf));
    REQUIRE(std::string(buf) == "sitl-injected-id");

    std::uint8_t unformatted[50] = {};
    std::uint8_t len = 50;
    REQUIRE(util.get_system_id_unformatted(unformatted, len));
    REQUIRE(len == std::strlen(raw));
    REQUIRE(std::string(reinterpret_cast<char*>(unformatted), len) == "sitl-injected-id");

    util.set_instance(1);
    char inst[50] = {};
    REQUIRE(util.get_system_id(inst));
    REQUIRE(inst[0] == static_cast<char>('s' + 1));
    REQUIRE(std::string(inst + 1) == "itl-injected-id");

    Util empty;
    char miss[50] = {};
    REQUIRE_FALSE(empty.get_system_id(miss));
}

TEST_CASE("available_memory is SITL 512k", "[hal][util][memory]") {
    Util util;
    REQUIRE(util.available_memory() == 512U * 1024U);
    REQUIRE(util.available_memory() == kSitlAvailableMemoryBytes);
}

TEST_CASE("leftover remaining_count is 1 after CAN slice", "[hal][util][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(completeness_has("Util", PortStatus::kOnMain));
    REQUIRE(completeness_has("CAN", PortStatus::kThisSlice));
    REQUIRE(completeness_has("WSPI", PortStatus::kRemaining));
}
