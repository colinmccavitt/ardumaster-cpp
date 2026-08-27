// Tests for CPP-045: PitchController::Gains's real top-level AP_Param
// Info[] table, defaults application, and a real save/load round-trip
// through the SAME storage::RawStorage/StorageAccess/ParamHeader
// machinery CPP-020/CPP-021/CPP-022 already built and verified
// (ADR-0013) - a storage/persistence integration test, NOT a SimPlane
// closed-loop test (per the ticket's own instruction). See
// pitch_controller.hpp's own "CPP-045 ADDENDUM" file banner for the full
// design rationale this test exercises: the real upstream GOBJECT prefix
// ("PTCH", not "PTCH2SRV_"), which 4 of Gains' 5 fields are genuinely
// upstream AP_Param-backed, the native-value bridge (not CPP-022 slice
// 6/7's ParamValue<T>-based functions), and why `rate_pid` (a real
// upstream AP_Param too, AP_SUBGROUPINFO index 11) is deliberately
// excluded from this ticket's table.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/fw_control/pitch_controller.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::fw_control;
using fwcpp::param::VarType;

TEST_CASE("PitchController::Gains's own default-constructed fields already carry the real upstream defaults (spot-check against AP_PitchController.cpp's real var_info[])", "[pitch][defaults]") {
    PitchController::Gains gains;
    // Every value below is grepped directly from
    // libraries/APM_Control/AP_PitchController.cpp's real var_info[] for
    // this ticket, not trusted from this file's own pre-existing comment
    // alone.
    REQUIRE(gains.tau == 0.5f);      // 2SRV_TCONST
    REQUIRE(gains.rmax_pos == 0.0f); // 2SRV_RMAX_UP
    REQUIRE(gains.rmax_neg == 0.0f); // 2SRV_RMAX_DN
    REQUIRE(gains.roll_ff == 1.0f);  // 2SRV_RLL (upstream's _roll_ff)
}

TEST_CASE("apply_pitch_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as Gains' own C++ initializers", "[pitch][defaults]") {
    PitchController::Gains gains;
    // Deliberately corrupt every real-backed field first, so this test
    // actually proves apply_pitch_defaults() writes them, rather than
    // passing vacuously because the constructor already got there.
    gains.tau = 1.0f;
    gains.rmax_pos = 1.0f;
    gains.rmax_neg = 1.0f;
    gains.roll_ff = 1.0f + 1.0f; // != its real default (1.0f)

    apply_pitch_defaults(gains);

    REQUIRE(gains.tau == 0.5f);
    REQUIRE(gains.rmax_pos == 0.0f);
    REQUIRE(gains.rmax_neg == 0.0f);
    REQUIRE(gains.roll_ff == 1.0f);
}

TEST_CASE("pitch_param_info's table matches upstream's real names/keys/types for every one of the 4 real Gains fields", "[pitch][info]") {
    PitchController::Gains gains;
    const auto table = pitch_param_info(gains);

    REQUIRE(std::string(table[0].name) == "PTCH2SRV_TCONST");
    REQUIRE(table[0].ptr == &gains.tau);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[0].def_value == 0.5f);
    REQUIRE(table[0].key == static_cast<std::uint16_t>(PitchParamKey::kTau));

    REQUIRE(std::string(table[1].name) == "PTCH2SRV_RMAX_UP");
    REQUIRE(table[1].ptr == &gains.rmax_pos);
    REQUIRE(table[1].def_value == 0.0f);

    REQUIRE(std::string(table[2].name) == "PTCH2SRV_RMAX_DN");
    REQUIRE(table[2].ptr == &gains.rmax_neg);
    REQUIRE(table[2].def_value == 0.0f);

    REQUIRE(std::string(table[3].name) == "PTCH2SRV_RLL");
    REQUIRE(table[3].ptr == &gains.roll_ff);
    REQUIRE(table[3].def_value == 1.0f);

    REQUIRE(table[4].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("top_level::find locates a real Gains field by its real upstream full name", "[pitch][find]") {
    PitchController::Gains gains;
    gains.tau = 0.75f;
    const auto table = pitch_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("PTCH2SRV_TCONST", table.data(), ptype);
    REQUIRE(p == &gains.tau);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 0.75f);

    // Case-insensitive, matching upstream's real top-level scalar match.
    REQUIRE(fwcpp::param::find("ptch2srv_rll", table.data(), ptype) == &gains.roll_ff);

    // Not present at all: a name with the real full "PTCH" prefix but no
    // matching entry in THIS ticket's table (e.g. the deferred rate_pid
    // subgroup's own real name) must not be found.
    REQUIRE(fwcpp::param::find("PTCH_RATE_P", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults", "[pitch][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- Gains A: mutate some real-backed fields away from default, save ---
    PitchController::Gains gains_a;
    gains_a.tau = 0.6f;         // default 0.5
    gains_a.roll_ff = 1.2f;     // default 1.0
    // rmax_pos, rmax_neg are left at their real defaults - untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_pitch_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's key must NOT be found in storage at all - it was never
    // written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(PitchParamKey::kRmaxPos)); // untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(PitchParamKey::kTau)); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));
    }

    // --- Gains B: fresh instance, load from the SAME backing storage ---
    PitchController::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_pitch_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.tau == 0.6f);
    REQUIRE(gains_b.roll_ff == 1.2f);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(gains_b.rmax_pos == 0.0f);
    REQUIRE(gains_b.rmax_neg == 0.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[pitch][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    PitchController::Gains gains_a; // rmax_pos left at its real default, 0.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_pitch_parameters(storage, gains_a, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(PitchParamKey::kRmaxPos));
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
