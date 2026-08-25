// Tests for fwcpp::filter::FilterRegistry (CPP-024 slice 2).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/filter_registry.hpp>

using namespace fwcpp::filter;

TEST_CASE("get_notch_filter returns nullptr for index 0 (matches upstream's 1-based contract)", "[filter_registry]") {
    FilterRegistry registry;
    REQUIRE(registry.get_notch_filter(0) == nullptr);
}

TEST_CASE("get_notch_filter returns nullptr for an out-of-range index", "[filter_registry]") {
    FilterRegistry registry;
    REQUIRE(registry.get_notch_filter(FilterRegistry::kNumFilters + 1) == nullptr);
    REQUIRE(registry.get_notch_filter(255) == nullptr);
}

TEST_CASE("get_notch_filter returns a valid pointer for every in-range 1-based index", "[filter_registry]") {
    FilterRegistry registry;
    for (std::uint8_t i = 1; i <= FilterRegistry::kNumFilters; ++i) {
        REQUIRE(registry.get_notch_filter(i) != nullptr);
    }
}

TEST_CASE("each slot is independently configurable and starts at upstream's own defaults", "[filter_registry]") {
    FilterRegistry registry;
    NotchFilterParams* slot1 = registry.get_notch_filter(1);
    NotchFilterParams* slot2 = registry.get_notch_filter(2);
    REQUIRE(slot1 != slot2);

    REQUIRE(slot1->center_freq_hz().get() == 0.0f);
    slot1->center_freq_hz_.set(80.0f);
    REQUIRE(slot2->center_freq_hz().get() == 0.0f); // unaffected by slot1's change
}
