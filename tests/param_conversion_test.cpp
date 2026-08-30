#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <fwcpp/param/conversion.hpp>
#include <fwcpp/param/conversion_leftover.hpp>
#include <fwcpp/param/param.hpp>

using Catch::Approx;
using fwcpp::param::ClassConversionInfo;
using fwcpp::param::ConversionInfo;
using fwcpp::param::G2ConversionEntry;
using fwcpp::param::NewParamStore;
using fwcpp::param::OldParamStore;
using fwcpp::param::ToplevelConversionEntry;
using fwcpp::param::VarType;
using fwcpp::param::WidthConvertEffects;
using fwcpp::param::WidthConvertInputs;
using fwcpp::param::kConvertFlagForce;
using fwcpp::param::kConvertFlagReverse;
using fwcpp::param::convert_old_parameter;
using fwcpp::param::convert_old_parameters;
using fwcpp::param::convert_old_parameters_scaled;
using fwcpp::param::find_old_parameter;
using fwcpp::param::leftover_convert_bitmask_parameter_width;
using fwcpp::param::leftover_convert_centi_parameter;
using fwcpp::param::leftover_convert_class;
using fwcpp::param::leftover_convert_g2;
using fwcpp::param::leftover_convert_parameter_width;
using fwcpp::param::leftover_convert_toplevel;
using fwcpp::param::new_store_find;
using fwcpp::param::new_store_put;
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

TEST_CASE("leftover_convert_parameter_width skips configured_in_storage",
          "[param][conversion][width]") {
    WidthConvertInputs in{};
    in.configured_in_storage = true;
    in.old_value_found = true;
    in.old_value = 5.0f;
    WidthConvertEffects fx{};
    REQUIRE_FALSE(leftover_convert_parameter_width(in, fx));
    REQUIRE(fx.skipped_configured);
    REQUIRE_FALSE(fx.converted);
    REQUIRE_FALSE(fx.skipped_missing);
}

TEST_CASE("leftover_convert_parameter_width skips missing old value",
          "[param][conversion][width]") {
    WidthConvertInputs in{};
    in.old_value_found = false;
    WidthConvertEffects fx{};
    REQUIRE_FALSE(leftover_convert_parameter_width(in, fx));
    REQUIRE(fx.skipped_missing);
    REQUIRE_FALSE(fx.converted);
    REQUIRE_FALSE(fx.skipped_configured);
}

TEST_CASE("leftover_convert_parameter_width scales non-bitmask",
          "[param][conversion][width]") {
    WidthConvertInputs in{};
    in.old_value_found = true;
    in.old_value = 100.0f;
    in.scale_factor = 0.01f;
    WidthConvertEffects fx{};
    REQUIRE(leftover_convert_parameter_width(in, fx));
    REQUIRE(fx.converted);
    REQUIRE(fx.new_value == Approx(1.0f));
    REQUIRE_FALSE(fx.skipped_configured);
    REQUIRE_FALSE(fx.skipped_missing);
}

TEST_CASE("leftover_convert_parameter_width bitmask uses simple uint32 cast",
          "[param][conversion][width]") {
    // Inject already holds a non-negative float; truncating cast through
    // uint32 is the documented stub (typed int8 -1 → 255 remain catalogued).
    WidthConvertInputs in{};
    in.old_value_found = true;
    in.old_value = 42.7f;
    in.bitmask = true;
    WidthConvertEffects fx{};
    REQUIRE(leftover_convert_parameter_width(in, fx));
    REQUIRE(fx.converted);
    REQUIRE(fx.new_value == 42.0f);
}

TEST_CASE("convert_old_parameter skips when new_configured without FORCE",
          "[param][conversion][flags]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 1, 0, static_cast<std::uint8_t>(VarType::Float), 10.0f));
    REQUIRE(new_store_put(neu, "KEEP", 99.0f));
    ConversionInfo info{1, 0, static_cast<std::uint8_t>(VarType::Float), "KEEP"};
    convert_old_parameter(info, 1.0f, 0, old, neu, /*new_configured=*/true);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "KEEP", out));
    REQUIRE(out == 99.0f);
}

TEST_CASE("convert_old_parameter FORCE overwrites configured new value",
          "[param][conversion][flags]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 1, 0, static_cast<std::uint8_t>(VarType::Float), 10.0f));
    REQUIRE(new_store_put(neu, "FORCED", 99.0f));
    ConversionInfo info{1, 0, static_cast<std::uint8_t>(VarType::Float), "FORCED"};
    convert_old_parameter(info, 0.5f, kConvertFlagForce, old, neu, /*new_configured=*/true);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "FORCED", out));
    REQUIRE(out == Approx(5.0f));
}

TEST_CASE("convert_old_parameter REVERSE maps _REV -1 to _REVERSED 1",
          "[param][conversion][flags]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 2, 0, static_cast<std::uint8_t>(VarType::Float), -1.0f));
    ConversionInfo info{2, 0, static_cast<std::uint8_t>(VarType::Float), "SERVO_REVERSED"};
    convert_old_parameter(info, 1.0f, kConvertFlagReverse, old, neu);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "SERVO_REVERSED", out));
    REQUIRE(out == 1.0f);
}

TEST_CASE("convert_old_parameter REVERSE maps other old values to 0",
          "[param][conversion][flags]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 2, 0, static_cast<std::uint8_t>(VarType::Float), 1.0f));
    ConversionInfo info{2, 0, static_cast<std::uint8_t>(VarType::Float), "SERVO_REVERSED"};
    convert_old_parameter(info, 1.0f, kConvertFlagReverse, old, neu);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "SERVO_REVERSED", out));
    REQUIRE(out == 0.0f);
}

TEST_CASE("convert_old_parameter REVERSE then scaler", "[param][conversion][flags]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 3, 0, static_cast<std::uint8_t>(VarType::Float), -1.0f));
    ConversionInfo info{3, 0, static_cast<std::uint8_t>(VarType::Float), "REV_SCALED"};
    convert_old_parameter(info, 2.0f, kConvertFlagReverse, old, neu);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "REV_SCALED", out));
    REQUIRE(out == Approx(2.0f)); // (is_equal(-1)?1:0) * 2.0
}

TEST_CASE("leftover_convert_class walks field table with shared old key",
          "[param][conversion][class]") {
    OldParamStore old{};
    NewParamStore neu{};
    constexpr std::uint16_t kOldClassKey = 42;
    REQUIRE(old_store_put(old, kOldClassKey, 0, static_cast<std::uint8_t>(VarType::Float), 1.5f));
    REQUIRE(old_store_put(old, kOldClassKey, 1, static_cast<std::uint8_t>(VarType::Float), 2.5f));
    REQUIRE(old_store_put(old, kOldClassKey, 2, static_cast<std::uint8_t>(VarType::Int16), 7.0f));
    const ClassConversionInfo table[] = {
        {0, static_cast<std::uint8_t>(VarType::Float), "CLS_A"},
        {1, static_cast<std::uint8_t>(VarType::Float), "CLS_B"},
        {2, static_cast<std::uint8_t>(VarType::Int16), "CLS_C"},
        {3, static_cast<std::uint8_t>(VarType::Float), "CLS_ABSENT"},
    };
    leftover_convert_class(kOldClassKey, table, 4, old, neu);
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float missing = 0.0f;
    REQUIRE(new_store_find(neu, "CLS_A", a));
    REQUIRE(a == Approx(1.5f));
    REQUIRE(new_store_find(neu, "CLS_B", b));
    REQUIRE(b == Approx(2.5f));
    REQUIRE(new_store_find(neu, "CLS_C", c));
    REQUIRE(c == 7.0f);
    REQUIRE_FALSE(new_store_find(neu, "CLS_ABSENT", missing));
}

TEST_CASE("leftover_convert_class skips when new_configured",
          "[param][conversion][class]") {
    OldParamStore old{};
    NewParamStore neu{};
    constexpr std::uint16_t kOldClassKey = 9;
    REQUIRE(old_store_put(old, kOldClassKey, 0, static_cast<std::uint8_t>(VarType::Float), 3.0f));
    REQUIRE(new_store_put(neu, "KEEP_CLS", 99.0f));
    const ClassConversionInfo table[] = {
        {0, static_cast<std::uint8_t>(VarType::Float), "KEEP_CLS"},
    };
    leftover_convert_class(kOldClassKey, table, 1, old, neu, /*new_configured=*/true);
    float out = 0.0f;
    REQUIRE(new_store_find(neu, "KEEP_CLS", out));
    REQUIRE(out == 99.0f);
}

TEST_CASE("leftover_convert_class null table is no-op", "[param][conversion][class]") {
    OldParamStore old{};
    NewParamStore neu{};
    leftover_convert_class(1, nullptr, 0, old, neu);
    REQUIRE(neu.count == 0);
}

TEST_CASE("leftover_convert_g2 loops entries into leftover_convert_class",
          "[param][conversion][g2]") {
    OldParamStore old{};
    NewParamStore neu{};
    constexpr std::uint16_t kG2Key = 55;
    REQUIRE(old_store_put(old, kG2Key, 0, static_cast<std::uint8_t>(VarType::Float), 1.0f));
    REQUIRE(old_store_put(old, kG2Key, 1, static_cast<std::uint8_t>(VarType::Float), 2.0f));
    REQUIRE(old_store_put(old, kG2Key, 10, static_cast<std::uint8_t>(VarType::Float), 3.0f));
    const ClassConversionInfo obj_a[] = {
        {0, static_cast<std::uint8_t>(VarType::Float), "G2_A0"},
        {1, static_cast<std::uint8_t>(VarType::Float), "G2_A1"},
    };
    const ClassConversionInfo obj_b[] = {
        {10, static_cast<std::uint8_t>(VarType::Float), "G2_B0"},
    };
    const G2ConversionEntry entries[] = {
        {obj_a, 2},
        {obj_b, 1},
    };
    leftover_convert_g2(kG2Key, entries, 2, old, neu);
    float a0 = 0.0f;
    float a1 = 0.0f;
    float b0 = 0.0f;
    REQUIRE(new_store_find(neu, "G2_A0", a0));
    REQUIRE(a0 == Approx(1.0f));
    REQUIRE(new_store_find(neu, "G2_A1", a1));
    REQUIRE(a1 == Approx(2.0f));
    REQUIRE(new_store_find(neu, "G2_B0", b0));
    REQUIRE(b0 == Approx(3.0f));
}

TEST_CASE("leftover_convert_g2 null entries is no-op", "[param][conversion][g2]") {
    OldParamStore old{};
    NewParamStore neu{};
    leftover_convert_g2(1, nullptr, 0, old, neu);
    REQUIRE(neu.count == 0);
}

TEST_CASE("leftover_convert_toplevel uses per-entry old_key",
          "[param][conversion][g2]") {
    OldParamStore old{};
    NewParamStore neu{};
    REQUIRE(old_store_put(old, 7, 0, static_cast<std::uint8_t>(VarType::Float), 4.0f));
    REQUIRE(old_store_put(old, 8, 0, static_cast<std::uint8_t>(VarType::Float), 5.0f));
    const ClassConversionInfo t7[] = {
        {0, static_cast<std::uint8_t>(VarType::Float), "TOP_7"},
    };
    const ClassConversionInfo t8[] = {
        {0, static_cast<std::uint8_t>(VarType::Float), "TOP_8"},
    };
    const ToplevelConversionEntry entries[] = {
        {7, t7, 1},
        {8, t8, 1},
    };
    leftover_convert_toplevel(entries, 2, old, neu);
    float v7 = 0.0f;
    float v8 = 0.0f;
    REQUIRE(new_store_find(neu, "TOP_7", v7));
    REQUIRE(v7 == Approx(4.0f));
    REQUIRE(new_store_find(neu, "TOP_8", v8));
    REQUIRE(v8 == Approx(5.0f));
}

TEST_CASE("leftover_convert_centi_parameter scales by 0.01",
          "[param][conversion][width]") {
    WidthConvertInputs in{};
    in.old_value_found = true;
    in.old_value = 250.0f;
    in.scale_factor = 99.0f; // ignored — centi forces 0.01
    WidthConvertEffects fx{};
    REQUIRE(leftover_convert_centi_parameter(in, fx));
    REQUIRE(fx.converted);
    REQUIRE(fx.new_value == Approx(2.5f));
}

TEST_CASE("leftover_convert_bitmask_parameter_width uses bitmask path",
          "[param][conversion][width]") {
    WidthConvertInputs in{};
    in.old_value_found = true;
    in.old_value = 15.9f;
    in.bitmask = false; // ignored — bitmask helper forces true
    WidthConvertEffects fx{};
    REQUIRE(leftover_convert_bitmask_parameter_width(in, fx));
    REQUIRE(fx.converted);
    REQUIRE(fx.new_value == 15.0f);
}

TEST_CASE("conversion leftover catalog remaining_count", "[param][conversion][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 10);
    REQUIRE(on_main_count() == 0);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("ConversionInfo", PortStatus::kThisSlice));
    REQUIRE(completeness_has("convert_old_parameters_scaled", PortStatus::kThisSlice));
    REQUIRE(completeness_has("_convert_parameter_width", PortStatus::kThisSlice));
    REQUIRE(completeness_has("convert_old_parameter REVERSE/FORCE", PortStatus::kThisSlice));
    REQUIRE(completeness_has("convert_class", PortStatus::kThisSlice));
    REQUIRE(completeness_has("convert_g2 / convert_toplevel objects", PortStatus::kThisSlice));
    REQUIRE(completeness_has("bitmask / centi width helpers", PortStatus::kThisSlice));
    REQUIRE(completeness_has("find_old_parameter EEPROM scan", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("flush after convert", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("AP_Param singleton / EEPROM", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("convert_g2 / convert_toplevel objects", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("bitmask / centi width helpers", PortStatus::kRemaining));
}
