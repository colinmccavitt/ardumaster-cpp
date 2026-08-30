#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fwcpp/param/conversion.hpp>
#include <fwcpp/param/conversion_leftover.hpp>
#include <fwcpp/param/param.hpp>

using Catch::Approx;
using fwcpp::param::ConversionInfo;
using fwcpp::param::NewParamStore;
using fwcpp::param::OldParamStore;
using fwcpp::param::VarType;
using fwcpp::param::convert_old_parameter;
using fwcpp::param::convert_old_parameters;
using fwcpp::param::convert_old_parameters_scaled;
using fwcpp::param::find_old_parameter;
using fwcpp::param::new_store_find;
using fwcpp::param::old_store_put;
using fwcpp::param::conversion::PortStatus;
using fwcpp::param::conversion::completeness_has;
using fwcpp::param::conversion::completeness_size;
using fwcpp::param::conversion::on_main_count;
using fwcpp::param::conversion::out_of_scope_count;
using fwcpp::param::conversion::remaining_count;
using fwcpp::param::conversion::this_slice_count;

TEST_CASE("find_old_parameter inject matches key/group/type", "[param][conversion]") {
    OldParamStore old{};
    REQUIRE(old_store_put(old, 7, 2, static_cast<std::uint8_t>(VarType::Float), 12.5f));
    ConversionInfo info{7, 2, static_cast<std::uint8_t>(VarType::Float), "NEW_FOO"};
    float v = 0.0f;
    REQUIRE(find_old_parameter(info, old, v));
    REQUIRE(v == 12.5f);

    ConversionInfo miss{7, 3, static_cast<std::uint8_t>(VarType::Float), "NEW_FOO"};
    REQUIRE_FALSE(find_old_parameter(miss, old, v));
}

TEST_CASE("convert_old_parameter applies scaler into NewParamStore", "[param][conversion]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 1, 0, static_cast<std::uint8_t>(VarType::Float), 10.0f));
    ConversionInfo info{1, 0, static_cast<std::uint8_t>(VarType::Float), "SCALED"};
    convert_old_parameter(info, 0.01f, 0, old, neu);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "SCALED", out));
    REQUIRE(out == Approx(0.1f));
}

TEST_CASE("convert_old_parameter skips missing old entries", "[param][conversion]") {
    OldParamStore old{};
    NewParamStore neu{};
    ConversionInfo info{9, 0, static_cast<std::uint8_t>(VarType::Int16), "MISSING"};
    convert_old_parameter(info, 1.0f, 0, old, neu);
    float out = 0.0f;
    REQUIRE_FALSE(new_store_find(neu, "MISSING", out));
}

TEST_CASE("convert_old_parameters_scaled loops table like upstream", "[param][conversion]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 3, 1, static_cast<std::uint8_t>(VarType::Float), 4.0f));
    REQUIRE(old_store_put(old, 4, 0, static_cast<std::uint8_t>(VarType::Float), 8.0f));
    const ConversionInfo table[] = {
        {3, 1, static_cast<std::uint8_t>(VarType::Float), "A"},
        {4, 0, static_cast<std::uint8_t>(VarType::Float), "B"},
        {5, 0, static_cast<std::uint8_t>(VarType::Float), "C"}, // absent
    };
    convert_old_parameters_scaled(table, 3, 2.0f, 0, old, neu);
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    REQUIRE(new_store_find(neu, "A", a));
    REQUIRE(a == 8.0f);
    REQUIRE(new_store_find(neu, "B", b));
    REQUIRE(b == 16.0f);
    REQUIRE_FALSE(new_store_find(neu, "C", c));
}

TEST_CASE("convert_old_parameters uses scaler 1.0f", "[param][conversion]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 2, 0, static_cast<std::uint8_t>(VarType::Float), 3.0f));
    const ConversionInfo table[] = {
        {2, 0, static_cast<std::uint8_t>(VarType::Float), "ONE"},
    };
    convert_old_parameters(table, 1, 0, old, neu);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "ONE", out));
    REQUIRE(out == 3.0f);
}

TEST_CASE("conversion leftover catalog remaining_count", "[param][conversion][leftover]") {
    REQUIRE(remaining_count() == 7);
    REQUIRE(this_slice_count() == 5);
    REQUIRE(on_main_count() == 0);
    REQUIRE(out_of_scope_count() == 1);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("ConversionInfo", PortStatus::kThisSlice));
    REQUIRE(completeness_has("convert_old_parameters_scaled", PortStatus::kThisSlice));
    REQUIRE(completeness_has("_convert_parameter_width", PortStatus::kRemaining));
    REQUIRE(completeness_has("bitmask / centi width helpers", PortStatus::kRemaining));
    REQUIRE(completeness_has("AP_Param singleton / EEPROM", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("_convert_parameter_width", PortStatus::kThisSlice));
}
