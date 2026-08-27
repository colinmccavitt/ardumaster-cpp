// Tests for fwcpp::param::native_cast_to_float/set_native_value (CPP-043)
// - the memcpy-based equivalents of cast_to_float/set_value (CPP-022
// slice 7 / defaults.hpp) for scalars stored in plain native C++ types
// (float, bool, ...) rather than this port's ParamValue<T>/ParamFloat
// wrapper classes. See native_value.hpp's own banner for why these are
// separate from cast_to_float/set_value rather than a shared function.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/native_value.hpp>
#include <fwcpp/param/param.hpp>

#include <cmath>
#include <cstdint>

using namespace fwcpp::param;

TEST_CASE("set_native_value/native_cast_to_float round-trip a native float", "[param][native_value]") {
    float live = 0.0f;
    set_native_value(VarType::Float, &live, 12.5f);
    REQUIRE(live == 12.5f);
    REQUIRE(native_cast_to_float(VarType::Float, &live) == 12.5f);
}

TEST_CASE("set_native_value/native_cast_to_float round-trip a native bool via VarType::Int8 (0.0f/1.0f only, matching AP_Param boolean usage)", "[param][native_value]") {
    bool live = false;
    set_native_value(VarType::Int8, &live, 1.0f);
    REQUIRE(live == true);
    REQUIRE(native_cast_to_float(VarType::Int8, &live) == 1.0f);

    set_native_value(VarType::Int8, &live, 0.0f);
    REQUIRE(live == false);
    REQUIRE(native_cast_to_float(VarType::Int8, &live) == 0.0f);
}

TEST_CASE("set_native_value truncates toward zero for Int8/Int16/Int32, matching set_value's own implicit narrowing", "[param][native_value]") {
    std::int8_t i8 = 0;
    set_native_value(VarType::Int8, &i8, 5.9f);
    REQUIRE(i8 == 5);

    std::int16_t i16 = 0;
    set_native_value(VarType::Int16, &i16, -1200.7f);
    REQUIRE(i16 == -1200);

    std::int32_t i32 = 0;
    set_native_value(VarType::Int32, &i32, 70000.2f);
    REQUIRE(i32 == 70000);

    REQUIRE(native_cast_to_float(VarType::Int8, &i8) == 5.0f);
    REQUIRE(native_cast_to_float(VarType::Int16, &i16) == -1200.0f);
    REQUIRE(native_cast_to_float(VarType::Int32, &i32) == 70000.0f);
}

TEST_CASE("native_cast_to_float returns NaN for None/Group/Vector3f, matching cast_to_float's own fallback", "[param][native_value]") {
    int dummy = 0;
    REQUIRE(std::isnan(native_cast_to_float(VarType::None, &dummy)));
    REQUIRE(std::isnan(native_cast_to_float(VarType::Group, &dummy)));
    REQUIRE(std::isnan(native_cast_to_float(VarType::Vector3f, &dummy)));
}
