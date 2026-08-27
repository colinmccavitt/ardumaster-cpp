// Tests for CPP-047: SteerController::Gains's real top-level AP_Param
// GroupInfo/Info table (STEER2SRV_ prefix), defaults application, and a
// real save/load round-trip through the SAME storage::RawStorage/
// StorageAccess/ParamHeader machinery CPP-020/CPP-021/CPP-022 already
// built and verified (ADR-0013) - a storage/persistence integration
// test, NOT a SimPlane closed-loop test (per the ticket's own
// instruction). See steer_controller.hpp's own "CPP-047 ADDENDUM" for
// the full design rationale this test exercises: why this object gets a
// real single GROUP-type top-level Info entry (matching upstream's real
// `GOBJECT(steerController, "STEER2SRV_", AP_SteerController)`), unlike
// CPP-043's aparm (which had no GROUP entry at all), and why every one
// of Gains' ten fields is genuinely upstream AP_Param-backed (full 1:1
// coverage, unlike aparm's partial 13-of-~50).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>
#include <fwcpp/steer_control/steer_controller.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::steer_control;
using fwcpp::param::VarType;

TEST_CASE("SteerController::Gains's own default-constructed fields already carry the real upstream var_info[] defaults",
          "[steer_control][param][defaults]") {
    // Every value below is transcribed directly from
    // libraries/APM_Control/AP_SteerController.cpp's real var_info[]
    // (AP_GROUPINFO entries), not trusted from Gains' own pre-existing
    // comments alone.
    SteerController::Gains gains;
    REQUIRE(gains.tau == 0.75f);          // STEER2SRV_TCONST
    REQUIRE(gains.k_p == 1.8f);           // STEER2SRV_P
    REQUIRE(gains.k_i == 0.2f);           // STEER2SRV_I
    REQUIRE(gains.k_d == 0.005f);         // STEER2SRV_D
    REQUIRE(gains.imax == 1500);          // STEER2SRV_IMAX
    REQUIRE(gains.minspeed == 1.0f);      // STEER2SRV_MINSPD
    REQUIRE(gains.k_ff == 0.0f);          // STEER2SRV_FF
    REQUIRE(gains.deratespeed == 0.0f);   // STEER2SRV_DRTSPD
    REQUIRE(gains.deratefactor == 10.0f); // STEER2SRV_DRTFCT
    REQUIRE(gains.mindegree == 4500.0f);  // STEER2SRV_DRTMIN
}

TEST_CASE("apply_steer_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as Gains' own C++ initializers",
          "[steer_control][param][defaults]") {
    SteerController::Gains gains;
    // Deliberately corrupt every field first, so this test actually
    // proves apply_steer_defaults() writes them, rather than passing
    // vacuously because the constructor already got there.
    gains.tau = 1.0f;
    gains.k_p = 1.0f;
    gains.k_i = 1.0f;
    gains.k_d = 1.0f;
    gains.imax = 1;
    gains.minspeed = 1.0f;
    gains.k_ff = 1.0f;
    gains.deratespeed = 1.0f;
    gains.deratefactor = 1.0f;
    gains.mindegree = 1.0f;

    apply_steer_defaults(gains);

    REQUIRE(gains.tau == 0.75f);
    REQUIRE(gains.k_p == 1.8f);
    REQUIRE(gains.k_i == 0.2f);
    REQUIRE(gains.k_d == 0.005f);
    REQUIRE(gains.imax == 1500);
    REQUIRE(gains.minspeed == 1.0f);
    REQUIRE(gains.k_ff == 0.0f);
    REQUIRE(gains.deratespeed == 0.0f);
    REQUIRE(gains.deratefactor == 10.0f);
    REQUIRE(gains.mindegree == 4500.0f);
}

TEST_CASE("steer_gains_group_info's table matches upstream's real AP_GROUPINFO names/idx/types for every one of the ten real fields",
          "[steer_control][param][info]") {
    const fwcpp::param::GroupInfo* table = steer_gains_group_info();

    REQUIRE(std::string(table[0].name) == "TCONST");
    REQUIRE(table[0].idx == 0);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));
    REQUIRE(table[0].def_value == 0.75f);

    REQUIRE(std::string(table[1].name) == "P");
    REQUIRE(table[1].idx == 1);
    REQUIRE(table[1].def_value == 1.8f);

    // idx 2 is genuinely absent from upstream's own var_info[] - the
    // table's next entry after "P" (idx 1) is "I" at idx 3, not idx 2.
    REQUIRE(std::string(table[2].name) == "I");
    REQUIRE(table[2].idx == 3);
    REQUIRE(table[2].def_value == 0.2f);

    REQUIRE(std::string(table[3].name) == "D");
    REQUIRE(table[3].idx == 4);
    REQUIRE(table[3].def_value == 0.005f);

    REQUIRE(std::string(table[4].name) == "IMAX");
    REQUIRE(table[4].idx == 5);
    REQUIRE(table[4].type == static_cast<std::uint8_t>(VarType::Int16)); // matches upstream's real AP_Int16 width
    REQUIRE(table[4].def_value == 1500.0f);

    REQUIRE(std::string(table[5].name) == "MINSPD");
    REQUIRE(table[5].idx == 6);
    REQUIRE(table[5].def_value == 1.0f);

    REQUIRE(std::string(table[6].name) == "FF");
    REQUIRE(table[6].idx == 7);
    REQUIRE(table[6].def_value == 0.0f);

    REQUIRE(std::string(table[7].name) == "DRTSPD");
    REQUIRE(table[7].idx == 8);
    REQUIRE(table[7].def_value == 0.0f);

    REQUIRE(std::string(table[8].name) == "DRTFCT");
    REQUIRE(table[8].idx == 9);
    REQUIRE(table[8].def_value == 10.0f);

    REQUIRE(std::string(table[9].name) == "DRTMIN");
    REQUIRE(table[9].idx == 10);
    REQUIRE(table[9].def_value == 4500.0f);

    REQUIRE(table[10].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("steer_param_info wraps the GroupInfo table as a single real GROUP-type top-level Info entry, matching upstream's real GOBJECT registration",
          "[steer_control][param][info]") {
    SteerController::Gains gains;
    const auto table = steer_param_info(gains);

    REQUIRE(std::string(table[0].name) == "STEER2SRV_");
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Group));
    REQUIRE(table[0].ptr == &gains);
    REQUIRE(table[0].group_info == steer_gains_group_info());
    REQUIRE(table[0].key == static_cast<std::uint16_t>(SteerParamKey::kSteerController));

    REQUIRE(table[1].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("top_level::find locates every real steering field by its real upstream STEER2SRV_ name",
          "[steer_control][param][find]") {
    SteerController::Gains gains;
    gains.k_p = 3.3f;
    const auto table = steer_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("STEER2SRV_P", table.data(), ptype);
    REQUIRE(p == &gains.k_p);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 3.3f);

    // Case-sensitive GROUP-prefix match, case-INsensitive leaf match -
    // matches upstream's own real (and genuinely inconsistent) find()/
    // find_group() case-sensitivity split, preserved exactly (see
    // top_level.hpp's own banner and name_lookup.hpp's own banner).
    REQUIRE(fwcpp::param::find("STEER2SRV_tconst", table.data(), ptype) == &gains.tau);
    REQUIRE(fwcpp::param::find("steer2srv_TCONST", table.data(), ptype) == nullptr); // wrong-case PREFIX fails

    REQUIRE(fwcpp::param::find("STEER2SRV_IMAX", table.data(), ptype) == &gains.imax);
    REQUIRE(ptype == VarType::Int16);
    REQUIRE(fwcpp::param::find("STEER2SRV_DRTMIN", table.data(), ptype) == &gains.mindegree);

    // A nonexistent field name (including the never-registered idx-2 gap)
    // never resolves.
    REQUIRE(fwcpp::param::find("STEER2SRV_BOGUS", table.data(), ptype) == nullptr);
    REQUIRE(fwcpp::param::find("BOGUS_PREFIX_P", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults",
          "[steer_control][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- gains_a: mutate several fields away from default, save ---
    SteerController::Gains gains_a;
    gains_a.tau = 0.5f;            // default 0.75
    gains_a.k_p = 2.5f;            // default 1.8
    gains_a.imax = 900;            // default 1500
    gains_a.deratefactor = 20.0f;  // default 10
    // k_i, k_d, minspeed, k_ff, deratespeed, mindegree are left at their
    // real defaults - untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_steer_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022): an untouched field's
    // (key, group_element) pair must NOT be found in storage at all - it
    // was never written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        const fwcpp::param::GroupInfo* table = steer_gains_group_info();

        // k_i (idx 3, untouched) -> not found.
        fwcpp::param::ParamHeader phdr_ki{};
        phdr_ki.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_ki, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr_ki.group_element = fwcpp::param::group_id(table, 0, 2, 0); // table[2] == "I", idx 3
        REQUIRE(phdr_ki.group_element == 3);
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr_ki, found_offset, sentinel_offset));

        // tau (idx 0, changed) -> found.
        fwcpp::param::ParamHeader phdr_tau{};
        phdr_tau.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_tau, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr_tau.group_element = fwcpp::param::group_id(table, 0, 0, 0); // table[0] == "TCONST", idx 0
        REQUIRE(fwcpp::param::scan(storage, phdr_tau, found_offset, sentinel_offset));

        // imax (idx 5, changed) -> found, confirming the Int16 field
        // round-trips through the default-skip path too.
        fwcpp::param::ParamHeader phdr_imax{};
        phdr_imax.type = static_cast<std::uint8_t>(VarType::Int16);
        fwcpp::param::set_key(phdr_imax, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr_imax.group_element = fwcpp::param::group_id(table, 0, 4, 0); // table[4] == "IMAX", idx 5
        REQUIRE(fwcpp::param::scan(storage, phdr_imax, found_offset, sentinel_offset));
    }

    // --- gains_b: fresh instance, load from the SAME backing storage ---
    SteerController::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_steer_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.tau == 0.5f);
    REQUIRE(gains_b.k_p == 2.5f);
    REQUIRE(gains_b.imax == 900);
    REQUIRE(gains_b.deratefactor == 20.0f);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(gains_b.k_i == 0.2f);
    REQUIRE(gains_b.k_d == 0.005f);
    REQUIRE(gains_b.minspeed == 1.0f);
    REQUIRE(gains_b.k_ff == 0.0f);
    REQUIRE(gains_b.deratespeed == 0.0f);
    REQUIRE(gains_b.mindegree == 4500.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[steer_control][param][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    SteerController::Gains gains; // k_ff left at its real default, 0.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_steer_parameters(storage, gains, /*force_save=*/true);

        const fwcpp::param::GroupInfo* table = steer_gains_group_info();
        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr.group_element = fwcpp::param::group_id(table, 0, 6, 0); // table[6] == "FF", idx 7
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
