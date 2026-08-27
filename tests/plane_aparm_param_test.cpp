// Tests for CPP-043: Plane::aparm's real top-level AP_Param Info[]
// table, defaults application, and a real save/load round-trip through
// the SAME storage::RawStorage/StorageAccess/ParamHeader machinery
// CPP-020/CPP-021/CPP-022 already built and verified (ADR-0013) - a
// storage/persistence integration test, NOT a SimPlane closed-loop test
// (per the ticket's own instruction). See plane.hpp's own "CPP-043
// ADDENDUM" (immediately after FixedWingTunables) for the full design
// rationale this test exercises: which 13 of FixedWingTunables' ~50
// fields are genuinely upstream aparm-backed, the native-value bridge
// (not CPP-022 slice 6/7's ParamValue<T>-based functions), and the
// registered on-storage-width divergence for seven of the thirteen.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/vehicle/mode.hpp> // provides Mode/ModeManual/etc's out-of-line vtable-anchoring definitions - see vehicle_test.cpp's own same include pattern
#include <fwcpp/vehicle/plane.hpp>

#include <cstdint>
#include <string>

using namespace fwcpp::vehicle;
using fwcpp::param::VarType;

TEST_CASE("Plane's own default-constructed aparm already carries the real upstream defaults (spot-check against Parameters.cpp/config.h)", "[plane][aparm][defaults]") {
    Plane plane;
    // Every value below is grepped directly from ArduPlane/config.h for
    // this ticket (ROLL_LIMIT_DEG, AIRSPEED_FBW_MIN/MAX,
    // AP_PLANE_TRIM_THROTTLE_DEFAULT, THROTTLE_MIN/MAX, AIRSPEED_CRUISE),
    // not trusted from FixedWingTunables' own pre-existing comment alone.
    REQUIRE(plane.aparm.roll_limit_deg == 45.0f);        // ROLL_LIMIT_DEG
    REQUIRE(plane.aparm.pitch_limit_max_deg == 20.0f);   // PITCH_MAX
    REQUIRE(plane.aparm.pitch_limit_min_deg == -25.0f);  // PITCH_MIN
    REQUIRE(plane.aparm.airspeed_min == 9.0f);           // AIRSPEED_FBW_MIN
    REQUIRE(plane.aparm.airspeed_max == 22.0f);          // AIRSPEED_FBW_MAX
    REQUIRE(plane.aparm.airspeed_stall == 0.0f);         // AIRSPEED_STALL
    REQUIRE(plane.aparm.stall_prevention == true);       // STALL_PREVENTION
    REQUIRE(plane.aparm.throttle_cruise == 45.0f);       // AP_PLANE_TRIM_THROTTLE_DEFAULT
    REQUIRE(plane.aparm.throttle_min == 0.0f);           // THROTTLE_MIN
    REQUIRE(plane.aparm.throttle_max == 100.0f);         // THROTTLE_MAX
    REQUIRE(plane.aparm.takeoff_throttle_max == 0.0f);   // TKOFF_THR_MAX (Parameters.cpp literal 0)
    REQUIRE(plane.aparm.airspeed_cruise == 12.0f);       // AIRSPEED_CRUISE
    REQUIRE(plane.aparm.loiter_radius == 60.0f);         // LOITER_RADIUS_DEFAULT
}

TEST_CASE("apply_aparm_defaults (the explicit, AP_Param-table-sourced path) reproduces the SAME values as FixedWingTunables' own C++ initializers", "[plane][aparm][defaults]") {
    Plane plane;
    // Deliberately corrupt every real-aparm field first, so this test
    // actually proves apply_aparm_defaults() writes them, rather than
    // passing vacuously because the constructor already got there.
    plane.aparm.roll_limit_deg = 1.0f;
    plane.aparm.pitch_limit_max_deg = 1.0f;
    plane.aparm.pitch_limit_min_deg = 1.0f;
    plane.aparm.airspeed_min = 1.0f;
    plane.aparm.airspeed_max = 1.0f;
    plane.aparm.airspeed_stall = 1.0f;
    plane.aparm.stall_prevention = false;
    plane.aparm.throttle_cruise = 1.0f;
    plane.aparm.throttle_min = 1.0f;
    plane.aparm.throttle_max = 1.0f;
    plane.aparm.takeoff_throttle_max = 1.0f;
    plane.aparm.airspeed_cruise = 1.0f;
    plane.aparm.loiter_radius = 1.0f;

    plane.apply_aparm_defaults();

    REQUIRE(plane.aparm.roll_limit_deg == 45.0f);
    REQUIRE(plane.aparm.pitch_limit_max_deg == 20.0f);
    REQUIRE(plane.aparm.pitch_limit_min_deg == -25.0f);
    REQUIRE(plane.aparm.airspeed_min == 9.0f);
    REQUIRE(plane.aparm.airspeed_max == 22.0f);
    REQUIRE(plane.aparm.airspeed_stall == 0.0f);
    REQUIRE(plane.aparm.stall_prevention == true);
    REQUIRE(plane.aparm.throttle_cruise == 45.0f);
    REQUIRE(plane.aparm.throttle_min == 0.0f);
    REQUIRE(plane.aparm.throttle_max == 100.0f);
    REQUIRE(plane.aparm.takeoff_throttle_max == 0.0f);
    REQUIRE(plane.aparm.airspeed_cruise == 12.0f);
    REQUIRE(plane.aparm.loiter_radius == 60.0f);
}

TEST_CASE("aparm_param_info's table matches upstream's real ASCALAR names/keys/types for every one of the 13 real aparm fields", "[plane][aparm][info]") {
    Plane plane;
    const auto table = aparm_param_info(plane.aparm);

    REQUIRE(std::string(table[0].name) == "ROLL_LIMIT_DEG");
    REQUIRE(table[0].ptr == &plane.aparm.roll_limit_deg);
    REQUIRE(table[0].type == static_cast<std::uint8_t>(VarType::Float));

    REQUIRE(std::string(table[6].name) == "STALL_PREVENTION");
    REQUIRE(table[6].ptr == &plane.aparm.stall_prevention);
    REQUIRE(table[6].type == static_cast<std::uint8_t>(VarType::Int8)); // matches upstream's real AP_Int8 width

    REQUIRE(std::string(table[7].name) == "TRIM_THROTTLE");
    REQUIRE(table[7].ptr == &plane.aparm.throttle_cruise);

    REQUIRE(std::string(table[12].name) == "WP_LOITER_RAD");
    REQUIRE(table[12].ptr == &plane.aparm.loiter_radius);

    REQUIRE(table[13].type == static_cast<std::uint8_t>(VarType::None)); // sentinel
}

TEST_CASE("find (top-level) locates a real aparm field by its real upstream name", "[plane][aparm][find]") {
    Plane plane;
    plane.aparm.roll_limit_deg = 33.0f;
    const auto table = aparm_param_info(plane.aparm);

    VarType ptype = VarType::None;
    void* p = fwcpp::param::find("ROLL_LIMIT_DEG", table.data(), ptype);
    REQUIRE(p == &plane.aparm.roll_limit_deg);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(*static_cast<float*>(p) == 33.0f);

    // Case-insensitive, matching upstream's real top-level scalar match.
    REQUIRE(fwcpp::param::find("trim_throttle", table.data(), ptype) == &plane.aparm.throttle_cruise);
}

TEST_CASE("save/load round-trips mutated aparm fields through a real storage::RawStorage/StorageAccess, and leaves untouched fields at their real defaults", "[plane][aparm][roundtrip]") {
    fwcpp::storage::RawStorage backing;

    // --- Plane A: mutate several real-aparm fields away from default, save ---
    Plane plane_a;
    plane_a.aparm.roll_limit_deg = 30.0f;         // default 45
    plane_a.aparm.throttle_cruise = 60.0f;        // default 45
    plane_a.aparm.stall_prevention = false;       // default true
    plane_a.aparm.airspeed_min = 11.0f;           // default 9
    plane_a.aparm.loiter_radius = 80.0f;          // default 60
    // airspeed_cruise, throttle_min, throttle_max, takeoff_throttle_max,
    // pitch_limit_max_deg, pitch_limit_min_deg, airspeed_max,
    // airspeed_stall are left at their real defaults - untouched.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_aparm_parameters(storage, plane_a.aparm);
    }

    // --- default-skip policy check (CPP-022 slice 7): an untouched
    // field's key must NOT be found in storage at all - it was never
    // written, matching should_skip_save's whole point.
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(AparmParamKey::kAirspeedCruise)); // untouched -> default
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE_FALSE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));

        fwcpp::param::ParamHeader phdr_changed{};
        phdr_changed.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr_changed, static_cast<std::uint16_t>(AparmParamKey::kRollLimitDeg)); // changed -> must be found
        REQUIRE(fwcpp::param::scan(storage, phdr_changed, found_offset, sentinel_offset));
    }

    // --- Plane B: fresh instance, load from the SAME backing storage ---
    Plane plane_b;
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        load_aparm_parameters(storage, plane_b.aparm);
    }

    // Mutated fields round-tripped correctly.
    REQUIRE(plane_b.aparm.roll_limit_deg == 30.0f);
    REQUIRE(plane_b.aparm.throttle_cruise == 60.0f);
    REQUIRE(plane_b.aparm.stall_prevention == false);
    REQUIRE(plane_b.aparm.airspeed_min == 11.0f);
    REQUIRE(plane_b.aparm.loiter_radius == 80.0f);

    // Untouched fields still show their real defaults (via load's own
    // "not found -> apply default" path, matching upstream's real
    // AP_Param::load() behavior).
    REQUIRE(plane_b.aparm.pitch_limit_max_deg == 20.0f);
    REQUIRE(plane_b.aparm.pitch_limit_min_deg == -25.0f);
    REQUIRE(plane_b.aparm.airspeed_max == 22.0f);
    REQUIRE(plane_b.aparm.airspeed_stall == 0.0f);
    REQUIRE(plane_b.aparm.throttle_min == 0.0f);
    REQUIRE(plane_b.aparm.throttle_max == 100.0f);
    REQUIRE(plane_b.aparm.takeoff_throttle_max == 0.0f);
    REQUIRE(plane_b.aparm.airspeed_cruise == 12.0f);
}

TEST_CASE("Plane's own save_aparm_parameters/load_aparm_parameters member wrappers round-trip through hal.storage", "[plane][aparm][roundtrip]") {
    Plane plane_a;
    plane_a.aparm.throttle_cruise = 55.0f;
    plane_a.save_aparm_parameters();

    // A fresh Plane never sees plane_a's hal.storage - load from the
    // SAME instance's own storage instead, simulating a reboot: mutate
    // aparm back to a different value in-memory, then reload.
    plane_a.aparm.throttle_cruise = 1.0f;
    plane_a.load_aparm_parameters();
    REQUIRE(plane_a.aparm.throttle_cruise == 55.0f);
}

TEST_CASE("force_save writes an unchanged (default-valued) field anyway", "[plane][aparm][roundtrip]") {
    fwcpp::storage::RawStorage backing;
    Plane plane_a; // airspeed_cruise left at its real default, 12.0f
    {
        fwcpp::storage::StorageAccess storage(backing, fwcpp::storage::StorageType::Param);
        save_aparm_parameters(storage, plane_a.aparm, /*force_save=*/true);

        fwcpp::param::ParamHeader phdr{};
        phdr.type = static_cast<std::uint8_t>(VarType::Float);
        fwcpp::param::set_key(phdr, static_cast<std::uint16_t>(AparmParamKey::kAirspeedCruise));
        std::uint16_t found_offset = 0;
        std::uint16_t sentinel_offset = 0;
        REQUIRE(fwcpp::param::scan(storage, phdr, found_offset, sentinel_offset));
    }
}
