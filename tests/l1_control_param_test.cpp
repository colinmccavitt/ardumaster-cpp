// Tests for CPP-048: L1Control::Gains's real top-level AP_Param Info[]
// table (NAVL1_ prefix), defaults application, and a real save/load
// round-trip through the SAME storage::RawStorage/StorageAccess/
// ParamHeader machinery CPP-020/CPP-021/CPP-022 already built and
// verified (ADR-0013) - a storage/persistence integration test, NOT a
// SimPlane closed-loop test (per the ticket's own instruction). See
// l1_control.hpp's own "CPP-048 ADDENDUM" (immediately after the
// L1Control class) for the full design rationale this test exercises:
// all four of Gains' fields are genuinely upstream AP_Param-backed (no
// exclusions, unlike CPP-043's aparm), the flat-top-level-table shape
// decision despite upstream's real GOBJECT/GROUP registration, the
// native-value bridge (not CPP-022 slice 6/7's ParamValue<T>-based
// functions), and a pre-existing port bug this ticket found but did NOT
// fix (Gains::l1_period's in-class default of 25.0f vs. the real
// upstream default of 17.0f).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/nav/l1_control.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::nav;
using fwcpp::param::VarType;

TEST_CASE("l1_param_info's table carries the REAL upstream AP_GROUPINFO defaults (AP_L1_Control.cpp, read directly)", "[l1][param][defaults]") {
    L1Control::Gains gains;
    const auto table = l1_param_info(gains);

    // Every value below is grepped directly from
    // libraries/AP_L1_Control/AP_L1_Control.cpp's var_info[] for this
    // ticket, NOT trusted from l1_control.hpp's own pre-existing
    // Gains{} in-class initializers (which this ticket found to be
    // WRONG for l1_period - see the addendum's own "PRE-EXISTING PORT
    // BUG" note. l1_period's real upstream default is 17, not 25).
    REQUIRE(table[0].def_value == 17.0f);   // NAVL1_PERIOD (real default 17, NOT Gains{}'s own 25.0f)
    REQUIRE(table[1].def_value == 0.75f);   // NAVL1_DAMPING
    REQUIRE(table[2].def_value == 0.02f);   // NAVL1_XTRACK_I
    REQUIRE(table[3].def_value == 0.0f);    // NAVL1_LIM_BANK

    // Documents, explicitly, that Gains{}'s own in-class default does
    // NOT match the real upstream default for l1_period - the
    // pre-existing divergence this ticket found and registered rather
    // than silently fixed (fixing it would ripple into
    // tests/vehicle_test.cpp's tuned numeric assertions, out of this
    // ticket's touch-scope).
    REQUIRE(gains.l1_period == 25.0f);
    REQUIRE(gains.l1_period != table[0].def_value);
}

TEST_CASE("apply_l1_defaults (the explicit, AP_Param-table-sourced path) writes the REAL upstream defaults", "[l1][param][defaults]") {
    L1Control::Gains gains;
    // Deliberately corrupt every field first, so this test actually
    // proves apply_l1_defaults() writes them, rather than passing
    // vacuously because Gains{}'s own initializers already got there
    // (and, for l1_period, they explicitly do NOT - see above).
    gains.l1_period = 1.0f;
    gains.l1_damping = 1.0f;
    gains.l1_xtrack_i_gain = 1.0f;
    gains.loiter_bank_limit = 1.0f;

    apply_l1_defaults(gains);

    REQUIRE(gains.l1_period == 17.0f);
    REQUIRE(gains.l1_damping == 0.75f);
    REQUIRE(gains.l1_xtrack_i_gain == 0.02f);
    REQUIRE(gains.loiter_bank_limit == 0.0f);
}

TEST_CASE("l1_param_info's table matches upstream's real AP_GROUPINFO names/keys/types for all four Gains fields", "[l1][param][info]") {
    L1Control::Gains gains;
    const auto table = l1_param_info(gains);

    REQUIRE(std::string(table[0].name) == "NAVL1_PERIOD");
    REQUIRE(table[0].ptr == &gains.l1_period);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[0].key == static_cast<std::uint16_t>(L1ParamKey::kPeriod));

    REQUIRE(std::string(table[1].name) == "NAVL1_DAMPING");
    REQUIRE(table[1].ptr == &gains.l1_damping);
    REQUIRE(table[1].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[1].key == static_cast<std::uint16_t>(L1ParamKey::kDamping));

    REQUIRE(std::string(table[2].name) == "NAVL1_XTRACK_I");
    REQUIRE(table[2].ptr == &gains.l1_xtrack_i_gain);
    REQUIRE(table[2].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[2].key == static_cast<std::uint16_t>(L1ParamKey::kXtrackIGain));

    REQUIRE(std::string(table[3].name) == "NAVL1_LIM_BANK");
    REQUIRE(table[3].ptr == &gains.loiter_bank_limit);
    REQUIRE(table[3].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[3].key == static_cast<std::uint16_t>(L1ParamKey::kLoiterBankLimit));

    REQUIRE(table[4].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("top_level::find locates a real Gains field by its real upstream NAVL1_ name", "[l1][param][find]") {
    L1Control::Gains gains;
    gains.l1_period = 33.0f;
    const auto table = l1_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("NAVL1_PERIOD", table.data(), ptype);
    REQUIRE(p == &gains.l1_period);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 33.0f);

    // Case-insensitive, matching upstream's real top-level scalar match
    // (top_level.hpp's own strcasecmp branch).
    REQUIRE(fwcpp::param::find("navl1_damping", table.data(), ptype) == &gains.l1_damping);

    // A name upstream would resolve through the GROUP-dispatch branch
    // instead (bare "PERIOD", without the "NAVL1_" prefix this ticket's
    // own flat table bakes into each entry's full name) correctly does
    // NOT resolve here - this table has no GROUP entry, by design (see
    // the addendum's own "REGISTERED DIVERGENCE" note).
    REQUIRE(fwcpp::param::find("PERIOD", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults", "[l1][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- Gains A: mutate two of the four real fields away from
    // default, save. The other two (l1_xtrack_i_gain, loiter_bank_limit)
    // are left untouched, at their real defaults. ---
    L1Control::Gains gains_a;
    apply_l1_defaults(gains_a); // start from the REAL defaults (17/0.75/0.02/0.0), not Gains{}'s own wrong 25.0f
    gains_a.l1_period = 20.0f;   // default 17
    gains_a.l1_damping = 0.85f;  // default 0.75
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_l1_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's key must NOT be found in storage at all - it was never
    // written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr_untouched{};
        phdr_untouched.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_untouched, static_cast<std::uint16_t>(L1ParamKey::kXtrackIGain)); // untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr_untouched, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_untouched2{};
        phdr_untouched2.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_untouched2, static_cast<std::uint16_t>(L1ParamKey::kLoiterBankLimit)); // untouched -> default
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr_untouched2, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(L1ParamKey::kPeriod)); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));
    }

    // --- Gains B: fresh instance, load from the SAME backing storage ---
    L1Control::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_l1_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.l1_period == 20.0f);
    REQUIRE(gains_b.l1_damping == 0.85f);

    // Untouched fields show their REAL defaults via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior - NOT Gains{}'s own in-class defaults
    // (which happen to already agree here, since only l1_period's
    // in-class default is wrong).
    REQUIRE(gains_b.l1_xtrack_i_gain == 0.02f);
    REQUIRE(gains_b.loiter_bank_limit == 0.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[l1][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    L1Control::Gains gains;
    apply_l1_defaults(gains); // loiter_bank_limit left at its real default, 0.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_l1_parameters(storage, gains, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(L1ParamKey::kLoiterBankLimit));
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
