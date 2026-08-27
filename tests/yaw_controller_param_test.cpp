// Tests for CPP-046: YawController::Gains's real top-level AP_Param
// Info[] table, defaults application, and a real save/load round-trip
// through the SAME storage::RawStorage/StorageAccess/ParamHeader
// machinery CPP-020/CPP-021/CPP-022 already built and verified
// (ADR-0013) - a storage/persistence integration test, NOT a SimPlane
// closed-loop test (per the ticket's own instruction), matching
// CPP-043's plane_aparm_param_test.cpp precedent. See
// yaw_controller.hpp's own "CPP-046 ADDENDUM" (immediately after the
// YawController class) for the full design rationale this test
// exercises: the real GOBJECT prefix correction ("YAW", not "YAW2SRV_"),
// which 6 of Gains' 7 fields are genuinely upstream AP_Param-backed, the
// native-value bridge (not CPP-022 slice 6/7's ParamValue<T>-based
// functions, since Gains' fields are plain float/bool), and the
// registered on-storage-width divergence for imax_cd.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/fw_control/yaw_controller.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::fw_control;
using fwcpp::param::VarType;

TEST_CASE("YawController::Gains's own default-constructed values already carry the real upstream var_info[] defaults",
          "[yaw][param][defaults]") {
    // Every value below is grepped directly from
    // libraries/APM_Control/AP_YawController.cpp's real var_info[] for
    // this ticket (AP_GROUPINFO's own trailing default argument), not
    // trusted from Gains' own pre-existing comment alone.
    YawController::Gains gains;
    REQUIRE(gains.k_a == 0.0f);          // 2SRV_SLIP, AP_GROUPINFO(..., _K_A, 0)
    REQUIRE(gains.k_i == 0.0f);          // 2SRV_INT,  AP_GROUPINFO(..., _K_I, 0)
    REQUIRE(gains.k_d == 0.0f);          // 2SRV_DAMP, AP_GROUPINFO(..., _K_D, 0)
    REQUIRE(gains.k_ff == 1.0f);         // 2SRV_RLL,  AP_GROUPINFO(..., _K_FF, 1)
    REQUIRE(gains.imax_cd == 1500.0f);   // 2SRV_IMAX, AP_GROUPINFO(..., _imax, 1500)
    REQUIRE(gains.rate_enable == false); // _RATE_ENABLE, AP_GROUPINFO_FLAGS(..., _rate_enable, 0, ...)
}

TEST_CASE("apply_yaw_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as Gains' own C++ initializers",
          "[yaw][param][defaults]") {
    YawController::Gains gains;
    // Deliberately corrupt every real field first, so this test actually
    // proves apply_yaw_defaults() writes them, rather than passing
    // vacuously because the struct's own initializers already got there.
    gains.k_a = 9.0f;
    gains.k_i = 9.0f;
    gains.k_d = 9.0f;
    gains.k_ff = 9.0f;
    gains.imax_cd = 9.0f;
    gains.rate_enable = true;

    apply_yaw_defaults(gains);

    REQUIRE(gains.k_a == 0.0f);
    REQUIRE(gains.k_i == 0.0f);
    REQUIRE(gains.k_d == 0.0f);
    REQUIRE(gains.k_ff == 1.0f);
    REQUIRE(gains.imax_cd == 1500.0f);
    REQUIRE(gains.rate_enable == false);
}

TEST_CASE("yaw_param_info's table matches upstream's real, prefix-concatenated names/keys/types for all 6 real fields, sentinel-terminated",
          "[yaw][param][info]") {
    YawController::Gains gains;
    const auto table = yaw_param_info(gains);

    REQUIRE(std::string(table[0].name) == "YAW2SRV_SLIP");
    REQUIRE(table[0].ptr == &gains.k_a);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));

    REQUIRE(std::string(table[1].name) == "YAW2SRV_INT");
    REQUIRE(table[1].ptr == &gains.k_i);

    REQUIRE(std::string(table[2].name) == "YAW2SRV_DAMP");
    REQUIRE(table[2].ptr == &gains.k_d);

    REQUIRE(std::string(table[3].name) == "YAW2SRV_RLL");
    REQUIRE(table[3].ptr == &gains.k_ff);

    REQUIRE(std::string(table[4].name) == "YAW2SRV_IMAX");
    REQUIRE(table[4].ptr == &gains.imax_cd);
    REQUIRE(table[4].type == static_cast<std::uint8_t>(VarType::Float)); // this port's own live width - see FINDING re: AP_Int16 divergence

    REQUIRE(std::string(table[5].name) == "YAW_RATE_ENABLE");
    REQUIRE(table[5].ptr == &gains.rate_enable);
    REQUIRE(table[5].type == static_cast<std::uint8_t>(VarType::Int8)); // matches upstream's real AP_Int8 width exactly
    REQUIRE((table[5].flags & fwcpp::param::kFlagEnable) != 0);         // real AP_PARAM_FLAG_ENABLE, preserved

    REQUIRE(table[6].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("top_level::find locates a real YawController::Gains field by its real, prefix-concatenated upstream name",
          "[yaw][param][find]") {
    YawController::Gains gains;
    gains.k_ff = 1.1f;
    const auto table = yaw_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("YAW2SRV_RLL", table.data(), ptype);
    REQUIRE(p == &gains.k_ff);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 1.1f);

    // Case-insensitive, matching upstream's real top-level scalar match
    // (top_level.hpp's own strcasecmp branch).
    REQUIRE(fwcpp::param::find("yaw2srv_imax", table.data(), ptype) == &gains.imax_cd);

    // A name that only differs by the (incorrect) "YAW2SRV_" prefix this
    // ticket's own scope text initially guessed at must NOT match -
    // confirms the real prefix really is "YAW", not "YAW2SRV_" (FINDING
    // #1): "YAW2SRV_2SRV_RLL" is not a real name, and searching for the
    // bare local name without any prefix must not match either.
    REQUIRE(fwcpp::param::find("2SRV_RLL", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults",
          "[yaw][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- gains_a: mutate several real fields away from default, save ---
    YawController::Gains gains_a;
    gains_a.k_a = 2.0f;           // default 0
    gains_a.k_d = 0.5f;           // default 0
    gains_a.k_ff = 1.05f;         // default 1
    gains_a.rate_enable = true;   // default false
    // k_i and imax_cd are left at their real defaults - untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_yaw_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's key must NOT be found in storage at all - it was never
    // written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(YawParamKey::kIntGain)); // untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(YawParamKey::kSlipGain)); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));
    }

    // --- gains_b: fresh instance, load from the SAME backing storage ---
    YawController::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_yaw_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.k_a == 2.0f);
    REQUIRE(gains_b.k_d == 0.5f);
    REQUIRE(gains_b.k_ff == 1.05f);
    REQUIRE(gains_b.rate_enable == true);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(gains_b.k_i == 0.0f);
    REQUIRE(gains_b.imax_cd == 1500.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[yaw][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    YawController::Gains gains_a; // k_i left at its real default, 0.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_yaw_parameters(storage, gains_a, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(YawParamKey::kIntGain));
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
