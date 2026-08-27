// Tests for CPP-049: Tecs::Gains's real AP_Param Info[]/GroupInfo[]
// table (TECS_ prefix, this port's existing fields only), defaults
// application, and a real save/load round-trip through the SAME
// storage::RawStorage/StorageAccess/ParamHeader machinery CPP-020/
// CPP-021/CPP-022 already built and verified (ADR-0013) - a storage/
// persistence integration test, NOT a SimPlane closed-loop test (per
// the ticket's own instruction). See tecs.hpp's own "CPP-049 ADDENDUM"
// (immediately after class Tecs's closing brace) for the full design
// rationale this test exercises: why TECS is a real GROUP ("TECS_",
// ArduPlane/Parameters.cpp:872's real GOBJECT) unlike CPP-043's flat
// aparm; which of Gains's 24 fields are genuinely upstream-backed (22)
// vs excluded (2 - the OPTIONS-bitmask-derived bools, no individually
// addressable real upstream entry); and the registered on-storage-width
// divergence for three of the 22 (pitch_max/pitch_min/
// thr_min_pct_ext_rate_lim - real upstream AP_Int8, this port's float).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>
#include <fwcpp/tecs/tecs.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::tecs;
using fwcpp::param::GroupInfo;
using fwcpp::param::VarType;

TEST_CASE("Tecs::Gains's own default-constructed values already carry the real upstream defaults (spot-check against AP_TECS.cpp's real var_info[])", "[tecs][param][defaults]") {
    Tecs::Gains gains;
    // Every value below is grepped directly from libraries/AP_TECS/
    // AP_TECS.cpp's real AP_GROUPINFO table for this ticket, not trusted
    // from Gains' own pre-existing field comment alone.
    REQUIRE(gains.max_climb_rate == 5.0f);            // CLMB_MAX
    REQUIRE(gains.min_sink_rate == 2.0f);             // SINK_MIN
    REQUIRE(gains.max_sink_rate == 5.0f);             // SINK_MAX
    REQUIRE(gains.time_const == 5.0f);                // TIME_CONST
    REQUIRE(gains.thr_damp == 0.5f);                  // THR_DAMP
    REQUIRE(gains.integ_gain == 0.3f);                // INTEG_GAIN
    REQUIRE(gains.vert_acc_lim == 7.0f);              // VERT_ACC
    REQUIRE(gains.hgt_comp_filt_omega == 3.0f);       // HGT_OMEGA
    REQUIRE(gains.spd_comp_filt_omega == 2.0f);       // SPD_OMEGA
    REQUIRE(gains.roll_comp == 10.0f);                // RLL2THR
    REQUIRE(gains.spd_weight == 1.0f);                // SPDWEIGHT
    REQUIRE(gains.ptch_damp == 0.3f);                 // PTCH_DAMP
    REQUIRE(gains.pitch_max == 15.0f);                // PITCH_MAX (real upstream AP_Int8, 15)
    REQUIRE(gains.pitch_min == 0.0f);                 // PITCH_MIN (real upstream AP_Int8, 0)
    REQUIRE(gains.use_synthetic_airspeed == false);   // SYNAIRSPEED
    REQUIRE(gains.pitch_ff_v0 == 12.0f);              // PTCH_FF_V0
    REQUIRE(gains.pitch_ff_k == 0.0f);                // PTCH_FF_K
    REQUIRE(gains.thr_min_pct_ext_rate_lim == 20.0f); // THR_ERATE (real upstream AP_Int8, 20)
    REQUIRE(gains.hgt_dem_tconst == 3.0f);             // HDEM_TCONST
    REQUIRE(gains.land_sink == 0.25f);                 // LAND_SINK
    REQUIRE(gains.land_sink_rate_change == 0.0f);      // LAND_SRC
    REQUIRE(gains.flare_holdoff_hgt == 1.0f);          // FLARE_HGT
}

TEST_CASE("apply_tecs_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as Gains' own C++ initializers", "[tecs][param][defaults]") {
    Tecs::Gains gains;
    // Deliberately corrupt every real field first (spd_weight to 2.0,
    // NOT 1.0 - its real default IS 1.0, so a mutation to the same value
    // would make that one field's check pass vacuously), so this test
    // actually proves apply_tecs_defaults() writes them, rather than
    // passing because the constructor already got there.
    gains.max_climb_rate = 1.0f;
    gains.min_sink_rate = 1.0f;
    gains.max_sink_rate = 1.0f;
    gains.time_const = 1.0f;
    gains.thr_damp = 1.0f;
    gains.integ_gain = 1.0f;
    gains.vert_acc_lim = 1.0f;
    gains.hgt_comp_filt_omega = 1.0f;
    gains.spd_comp_filt_omega = 1.0f;
    gains.roll_comp = 1.0f;
    gains.spd_weight = 2.0f;
    gains.ptch_damp = 1.0f;
    gains.pitch_max = 1.0f;
    gains.pitch_min = 1.0f;
    gains.use_synthetic_airspeed = true;
    gains.pitch_ff_v0 = 1.0f;
    gains.pitch_ff_k = 1.0f;
    gains.thr_min_pct_ext_rate_lim = 1.0f;
    gains.hgt_dem_tconst = 1.0f;
    gains.land_sink = 1.0f;
    gains.land_sink_rate_change = 1.0f;
    gains.flare_holdoff_hgt = 1.0f;

    apply_tecs_defaults(gains);

    REQUIRE(gains.max_climb_rate == 5.0f);
    REQUIRE(gains.min_sink_rate == 2.0f);
    REQUIRE(gains.max_sink_rate == 5.0f);
    REQUIRE(gains.time_const == 5.0f);
    REQUIRE(gains.thr_damp == 0.5f);
    REQUIRE(gains.integ_gain == 0.3f);
    REQUIRE(gains.vert_acc_lim == 7.0f);
    REQUIRE(gains.hgt_comp_filt_omega == 3.0f);
    REQUIRE(gains.spd_comp_filt_omega == 2.0f);
    REQUIRE(gains.roll_comp == 10.0f);
    REQUIRE(gains.spd_weight == 1.0f);
    REQUIRE(gains.ptch_damp == 0.3f);
    REQUIRE(gains.pitch_max == 15.0f);
    REQUIRE(gains.pitch_min == 0.0f);
    REQUIRE(gains.use_synthetic_airspeed == false);
    REQUIRE(gains.pitch_ff_v0 == 12.0f);
    REQUIRE(gains.pitch_ff_k == 0.0f);
    REQUIRE(gains.thr_min_pct_ext_rate_lim == 20.0f);
    REQUIRE(gains.hgt_dem_tconst == 3.0f);
    REQUIRE(gains.land_sink == 0.25f);
    REQUIRE(gains.land_sink_rate_change == 0.0f);
    REQUIRE(gains.flare_holdoff_hgt == 1.0f);
}

TEST_CASE("tecs_group_info's table matches upstream's real AP_GROUPINFO names/idx/types/offsets for every one of the 22 real Gains fields", "[tecs][param][info]") {
    Tecs::Gains gains;
    const GroupInfo* table = tecs_group_info();
    const auto* base = reinterpret_cast<const char*>(&gains);

    REQUIRE(std::string(table[0].name) == "CLMB_MAX");
    REQUIRE(table[0].idx == static_cast<std::uint8_t>(TecsGroupIdx::kClmbMax));
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(base + table[0].offset == reinterpret_cast<const char*>(&gains.max_climb_rate));

    REQUIRE(std::string(table[11].name) == "SINK_MAX");
    REQUIRE(table[11].idx == static_cast<std::uint8_t>(TecsGroupIdx::kSinkMax));
    REQUIRE(base + table[11].offset == reinterpret_cast<const char*>(&gains.max_sink_rate));

    REQUIRE(std::string(table[12].name) == "PITCH_MAX");
    REQUIRE(table[12].idx == static_cast<std::uint8_t>(TecsGroupIdx::kPitchMax));
    REQUIRE(table[12].type == static_cast<std::uint8_t>(VarType::Float)); // FINDING #3: real upstream AP_Int8, this port's own live float width
    REQUIRE(base + table[12].offset == reinterpret_cast<const char*>(&gains.pitch_max));

    REQUIRE(std::string(table[16].name) == "SYNAIRSPEED");
    REQUIRE(table[16].idx == static_cast<std::uint8_t>(TecsGroupIdx::kSynAirspeed));
    REQUIRE(table[16].type == static_cast<std::uint8_t>(VarType::Int8)); // matches upstream's real AP_Int8 width exactly
    REQUIRE(base + table[16].offset == reinterpret_cast<const char*>(&gains.use_synthetic_airspeed));

    REQUIRE(std::string(table[21].name) == "HDEM_TCONST");
    REQUIRE(table[21].idx == static_cast<std::uint8_t>(TecsGroupIdx::kHdemTconst));
    REQUIRE(base + table[21].offset == reinterpret_cast<const char*>(&gains.hgt_dem_tconst));

    REQUIRE(table[22].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("tecs_param_info's single top-level entry is a real GROUP matching ArduPlane/Parameters.cpp's real GOBJECT(TECS_controller, \"TECS_\", AP_TECS)", "[tecs][param][info]") {
    Tecs::Gains gains;
    const auto table = tecs_param_info(gains);

    REQUIRE(std::string(table[0].name) == "TECS_");
    REQUIRE(table[0].ptr == &gains);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Group));
    REQUIRE(table[0].group_info == tecs_group_info());
    REQUIRE(table[0].key == static_cast<std::uint16_t>(TecsParamKey::kTecsController));

    REQUIRE(table[1].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("OPTIONS-derived fields (option_glider_only/option_descent_speedup) are intentionally absent - no single addressable real upstream OPTIONS entry", "[tecs][param][info]") {
    Tecs::Gains gains;
    const auto table = tecs_param_info(gains);
    VarType ptype = VarType::None;
    // Neither the real upstream name nor any invented per-bit name
    // resolves - see tecs.hpp's CPP-049 ADDENDUM FINDING #2 for why this
    // is a real, disclosed exclusion, not an oversight: upstream's real
    // OPTIONS is a single AP_Int32 bitmask, but this port's Gains
    // (built before this ticket) already decomposed it into two
    // separate bool fields with no single addressable object to point
    // an Info/GroupInfo entry at.
    REQUIRE(fwcpp::param::find("TECS_OPTIONS", table.data(), ptype) == nullptr);
}

TEST_CASE("find (top-level) locates a real Gains field by its real upstream TECS_-prefixed name, via the real GROUP-dispatch branch", "[tecs][param][find]") {
    Tecs::Gains gains;
    gains.max_climb_rate = 7.0f;
    const auto table = tecs_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("TECS_CLMB_MAX", table.data(), ptype);
    REQUIRE(p == &gains.max_climb_rate);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 7.0f);

    // Case-INsensitive field-name match beneath a matched GROUP prefix
    // (find_group's own strcasecmp, name_lookup.hpp) - matches upstream's
    // real, documented behavior.
    REQUIRE(fwcpp::param::find("TECS_synairspeed", table.data(), ptype) == &gains.use_synthetic_airspeed);

    // The GROUP prefix match itself is case-SENSITIVE (top_level::find's
    // own strncmp, top_level.hpp) - a lowercase "tecs_" prefix must fail
    // to find anything, proving this is really exercising the real
    // asymmetric case-sensitivity upstream ships, not a case-insensitive
    // shortcut.
    REQUIRE(fwcpp::param::find("tecs_CLMB_MAX", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults", "[tecs][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- Gains A: mutate several real fields away from default, save ---
    Tecs::Gains gains_a;
    gains_a.max_climb_rate = 8.0f;          // default 5
    gains_a.time_const = 6.0f;              // default 5
    gains_a.use_synthetic_airspeed = true;  // default false
    gains_a.pitch_max = 20.0f;              // default 15
    gains_a.land_sink = 0.4f;               // default 0.25
    // Every other real field (min_sink_rate, max_sink_rate, thr_damp,
    // integ_gain, vert_acc_lim, hgt_comp_filt_omega,
    // spd_comp_filt_omega, roll_comp, spd_weight, ptch_damp, pitch_min,
    // pitch_ff_v0, pitch_ff_k, thr_min_pct_ext_rate_lim, hgt_dem_tconst,
    // land_sink_rate_change, flare_holdoff_hgt) is left untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_tecs_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's (key, group_element) pair must NOT be found in storage at
    // all - it was never written, matching should_skip_save's whole
    // point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr_untouched{};
        phdr_untouched.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_untouched, static_cast<std::uint16_t>(TecsParamKey::kTecsController));
        phdr_untouched.group_element = static_cast<std::uint32_t>(TecsGroupIdx::kSinkMin); // min_sink_rate, untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr_untouched, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(TecsParamKey::kTecsController));
        phdr_changed.group_element = static_cast<std::uint32_t>(TecsGroupIdx::kClmbMax); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed_int8{};
        phdr_changed_int8.type = static_cast<std::uint8_t>(VarType::Int8);
        fwcpp::param::set_key(phdr_changed_int8, static_cast<std::uint16_t>(TecsParamKey::kTecsController));
        phdr_changed_int8.group_element = static_cast<std::uint32_t>(TecsGroupIdx::kSynAirspeed); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed_int8, found_offset, sentinel_offset));
    }

    // --- Gains B: fresh instance, load from the SAME backing storage ---
    Tecs::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_tecs_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.max_climb_rate == 8.0f);
    REQUIRE(gains_b.time_const == 6.0f);
    REQUIRE(gains_b.use_synthetic_airspeed == true);
    REQUIRE(gains_b.pitch_max == 20.0f);
    REQUIRE(gains_b.land_sink == 0.4f);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(gains_b.min_sink_rate == 2.0f);
    REQUIRE(gains_b.max_sink_rate == 5.0f);
    REQUIRE(gains_b.thr_damp == 0.5f);
    REQUIRE(gains_b.integ_gain == 0.3f);
    REQUIRE(gains_b.vert_acc_lim == 7.0f);
    REQUIRE(gains_b.hgt_comp_filt_omega == 3.0f);
    REQUIRE(gains_b.spd_comp_filt_omega == 2.0f);
    REQUIRE(gains_b.roll_comp == 10.0f);
    REQUIRE(gains_b.spd_weight == 1.0f);
    REQUIRE(gains_b.ptch_damp == 0.3f);
    REQUIRE(gains_b.pitch_min == 0.0f);
    REQUIRE(gains_b.pitch_ff_v0 == 12.0f);
    REQUIRE(gains_b.pitch_ff_k == 0.0f);
    REQUIRE(gains_b.thr_min_pct_ext_rate_lim == 20.0f);
    REQUIRE(gains_b.hgt_dem_tconst == 3.0f);
    REQUIRE(gains_b.land_sink_rate_change == 0.0f);
    REQUIRE(gains_b.flare_holdoff_hgt == 1.0f);

    // OPTIONS-derived fields are untouched by load/save entirely (not in
    // scope, see FINDING #2) - still whatever Gains's own C++ default
    // member initializer gives a fresh instance.
    REQUIRE(gains_b.option_glider_only == false);
    REQUIRE(gains_b.option_descent_speedup == false);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[tecs][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    Tecs::Gains gains_a; // pitch_ff_k left at its real default, 0.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_tecs_parameters(storage, gains_a, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(TecsParamKey::kTecsController));
        phdr.group_element = static_cast<std::uint32_t>(TecsGroupIdx::kPtchFfK);
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
