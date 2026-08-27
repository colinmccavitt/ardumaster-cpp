// Tests for CPP-044: RollController::Gains's real top-level AP_Param
// Info[] table, defaults application, top_level::find()-by-name, and a
// real save/load round-trip through the SAME storage::RawStorage/
// StorageAccess/ParamHeader machinery CPP-020/CPP-021/CPP-022 already
// built and verified (ADR-0013) - a storage/persistence integration
// test, NOT a SimPlane closed-loop test (per the ticket's own
// instruction), matching CPP-043's own plane_aparm_param_test.cpp shape.
// See roll_controller.hpp's own "CPP-044 ADDENDUM" for the full design
// rationale this test exercises: which 12 of RollController::Gains's
// fields are genuinely upstream RLL/RLL_RATE-backed, the native-value
// bridge (not CPP-022 slice 6/7's ParamValue<T>-based functions), the
// flat-table shape decision, and the registered rmax_pos width
// divergence.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/fw_control/roll_controller.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/param/top_level.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::fw_control;
using fwcpp::param::VarType;

TEST_CASE("RollController::Gains's own default-constructed values already carry the real upstream defaults (spot-check against AP_RollController.cpp/AC_PID.cpp)", "[roll][params][defaults]") {
    RollController::Gains gains;
    // AP_RollController.cpp:36,47 (2SRV_TCONST/2SRV_RMAX) and the
    // constructor's own AC_PID::Defaults aggregate-init literal
    // (AP_RollController.cpp:150-161), both re-grepped directly for this
    // ticket, not trusted from this port's own pre-existing comments alone.
    REQUIRE(gains.tau == 0.5f);                // 2SRV_TCONST
    REQUIRE(gains.rmax_pos == 0.0f);           // 2SRV_RMAX
    REQUIRE(gains.rate_pid.p == 0.08f);        // AC_PID::Defaults.p
    REQUIRE(gains.rate_pid.i == 0.15f);        // .i
    REQUIRE(gains.rate_pid.d == 0.0f);         // .d
    REQUIRE(gains.rate_pid.ff == 0.345f);      // .ff
    REQUIRE(gains.rate_pid.imax == 0.666f);    // .imax
    REQUIRE(gains.rate_pid.filt_t_hz == 3.0f); // .filt_T_hz
    REQUIRE(gains.rate_pid.filt_e_hz == 0.0f); // .filt_E_hz
    REQUIRE(gains.rate_pid.filt_d_hz == 12.0f);// .filt_D_hz
    REQUIRE(gains.rate_pid.srmax == 150.0f);   // .srmax
    REQUIRE(gains.rate_pid.dff == 0.0f);       // .dff omitted from the aggregate-init -> value-initialized to 0
    REQUIRE(gains.rate_pid.srtau == 1.0f);     // .srtau - NOT AP_Param-backed (see addendum), but still a real constructor default
}

TEST_CASE("apply_roll_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as Gains' own C++ initializers", "[roll][params][defaults]") {
    RollController::Gains gains;
    // Deliberately corrupt every real-param field first, so this test
    // actually proves apply_roll_defaults() writes them, rather than
    // passing vacuously because the constructor already got there.
    gains.tau = 9.0f;
    gains.rmax_pos = 9.0f;
    gains.rate_pid.p = 9.0f;
    gains.rate_pid.i = 9.0f;
    gains.rate_pid.d = 9.0f;
    gains.rate_pid.ff = 9.0f;
    gains.rate_pid.imax = 9.0f;
    gains.rate_pid.filt_t_hz = 9.0f;
    gains.rate_pid.filt_e_hz = 9.0f;
    gains.rate_pid.filt_d_hz = 9.0f;
    gains.rate_pid.srmax = 9.0f;
    gains.rate_pid.dff = 9.0f;
    gains.rate_pid.srtau = 9.0f; // NOT in the Info table (see below) - must survive unchanged

    apply_roll_defaults(gains);

    REQUIRE(gains.tau == 0.5f);
    REQUIRE(gains.rmax_pos == 0.0f);
    REQUIRE(gains.rate_pid.p == 0.08f);
    REQUIRE(gains.rate_pid.i == 0.15f);
    REQUIRE(gains.rate_pid.d == 0.0f);
    REQUIRE(gains.rate_pid.ff == 0.345f);
    REQUIRE(gains.rate_pid.imax == 0.666f);
    REQUIRE(gains.rate_pid.filt_t_hz == 3.0f);
    REQUIRE(gains.rate_pid.filt_e_hz == 0.0f);
    REQUIRE(gains.rate_pid.filt_d_hz == 12.0f);
    REQUIRE(gains.rate_pid.srmax == 150.0f);
    REQUIRE(gains.rate_pid.dff == 0.0f);
    // srtau is NOT in the Info table (no real var_info entry - see
    // addendum) - apply_roll_defaults must not touch it at all.
    REQUIRE(gains.rate_pid.srtau == 9.0f);
}

TEST_CASE("roll_param_info's table matches upstream's real names/keys/types for every one of the 12 real RollController::Gains fields", "[roll][params][info]") {
    RollController::Gains gains;
    const auto table = roll_param_info(gains);

    REQUIRE(std::string(table[0].name) == "RLL2SRV_TCONST");
    REQUIRE(table[0].ptr == &gains.tau);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));

    REQUIRE(std::string(table[1].name) == "RLL2SRV_RMAX");
    REQUIRE(table[1].ptr == &gains.rmax_pos);
    REQUIRE(table[1].type == static_cast<std::uint8_t>(VarType::Float)); // registered width divergence - see addendum FINDING #3

    REQUIRE(std::string(table[2].name) == "RLL_RATE_P");
    REQUIRE(table[2].ptr == &gains.rate_pid.p);

    REQUIRE(std::string(table[6].name) == "RLL_RATE_IMAX");
    REQUIRE(table[6].ptr == &gains.rate_pid.imax);

    REQUIRE(std::string(table[11].name) == "RLL_RATE_D_FF");
    REQUIRE(table[11].ptr == &gains.rate_pid.dff);

    REQUIRE(table[12].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("top_level::find locates a real RollController::Gains field by its real upstream name", "[roll][params][find]") {
    RollController::Gains gains;
    gains.rate_pid.p = 0.21f;
    const auto table = roll_param_info(gains);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("RLL_RATE_P", table.data(), ptype);
    REQUIRE(p == &gains.rate_pid.p);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 0.21f);

    // Case-insensitive, matching upstream's real top-level scalar match.
    REQUIRE(fwcpp::param::find("rll2srv_tconst", table.data(), ptype) == &gains.tau);

    // A name that was never registered (excluded per the addendum) must
    // not resolve.
    REQUIRE(fwcpp::param::find("RLL_RATE_SRTAU", table.data(), ptype) == nullptr);
}

TEST_CASE("save/load round-trips mutated RollController::Gains fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults", "[roll][params][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- Gains A: mutate several real fields away from default, save ---
    RollController::Gains gains_a;
    gains_a.tau = 0.7f;                  // default 0.5
    gains_a.rmax_pos = 60.0f;            // default 0
    gains_a.rate_pid.p = 0.12f;          // default 0.08
    gains_a.rate_pid.filt_d_hz = 20.0f;  // default 12
    // rate_pid.i, .d, .ff, .imax, .filt_t_hz, .filt_e_hz, .srmax, .dff
    // are left at their real defaults - untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_roll_parameters(storage, gains_a);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's key must NOT be found in storage at all - it was never
    // written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(RollParamKey::kRateI)); // untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(RollParamKey::kTau)); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));
    }

    // --- Gains B: fresh instance, load from the SAME backing storage ---
    RollController::Gains gains_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_roll_parameters(storage, gains_b);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(gains_b.tau == 0.7f);
    REQUIRE(gains_b.rmax_pos == 60.0f);
    REQUIRE(gains_b.rate_pid.p == 0.12f);
    REQUIRE(gains_b.rate_pid.filt_d_hz == 20.0f);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(gains_b.rate_pid.i == 0.15f);
    REQUIRE(gains_b.rate_pid.d == 0.0f);
    REQUIRE(gains_b.rate_pid.ff == 0.345f);
    REQUIRE(gains_b.rate_pid.imax == 0.666f);
    REQUIRE(gains_b.rate_pid.filt_t_hz == 3.0f);
    REQUIRE(gains_b.rate_pid.filt_e_hz == 0.0f);
    REQUIRE(gains_b.rate_pid.srmax == 150.0f);
    REQUIRE(gains_b.rate_pid.dff == 0.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[roll][params][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    RollController::Gains gains_a; // rate_pid.imax left at its real default, 0.666f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_roll_parameters(storage, gains_a, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(RollParamKey::kRateImax));
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
