// Tests for fwcpp::param::find (CPP-043) - the top-level Info[] table
// walk that dispatches into the already-ported find_group (CPP-022
// slice 3) for GROUP-type entries, or matches directly for SCALAR-type
// entries. Synthetic vehicle/Info table, matching upstream's own
// AP_Param/tests/test_find_by_name.cpp pattern (a fake vehicle, not a
// real one) and this port's own established precedent (name_lookup_test.cpp,
// group_info_test.cpp).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/name_lookup.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/top_level.hpp>

#include <cstddef>

using namespace fwcpp::param;

namespace {
// A synthetic "sub-object" reachable only through a GROUP-type top-level
// Info entry - exercises the branch Plane's own real aparm Info table
// (fwcpp/vehicle/plane.hpp) does NOT exercise (aparm's real upstream
// fields are each individually top-level per ASCALAR, not grouped under
// one shared prefix - see plane.hpp's own "CPP-043 ADDENDUM" for why),
// so this synthetic table is what actually proves the GROUP branch
// works, independent of Plane.
struct SubObject {
    ParamFloat p;
    ParamFloat i;
};

GroupInfo sub_table[] = {
    [] {
        GroupInfo g{};
        g.name = "P";
        g.offset = offsetof(SubObject, p);
        g.type = static_cast<std::uint8_t>(VarType::Float);
        return g;
    }(),
    [] {
        GroupInfo g{};
        g.name = "I";
        g.offset = offsetof(SubObject, i);
        g.idx = 1;
        g.type = static_cast<std::uint8_t>(VarType::Float);
        return g;
    }(),
    [] {
        GroupInfo g{};
        g.type = static_cast<std::uint8_t>(VarType::None);
        return g;
    }(),
};

struct Vehicle {
    ParamFloat format_version;
    SubObject pid;
};

Vehicle vehicle{};

Info vehicle_info[] = {
    [] {
        Info info{};
        info.name = "FORMAT_VERSION";
        info.ptr = &vehicle.format_version;
        info.def_value = 0.0f;
        info.key = 1;
        info.type = static_cast<std::uint8_t>(VarType::Float);
        return info;
    }(),
    [] {
        Info info{};
        info.name = "PID_"; // trailing separator baked into the group's own name, matching name_lookup_test.cpp's own established convention
        info.ptr = &vehicle.pid;
        info.group_info = sub_table;
        info.key = 2;
        info.type = static_cast<std::uint8_t>(VarType::Group);
        return info;
    }(),
    [] {
        Info info{};
        info.type = static_cast<std::uint8_t>(VarType::None);
        return info;
    }(),
};
} // namespace

TEST_CASE("find locates a top-level SCALAR entry by exact (case-insensitive) name", "[param][find]") {
    vehicle.format_version.set(42.0f);
    VarType ptype = VarType::None;

    void* p = find("format_version", vehicle_info, ptype);
    REQUIRE(p == &vehicle.format_version);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(static_cast<ParamFloat*>(p)->get() == 42.0f);
}

TEST_CASE("find dispatches into find_group for a GROUP-type entry, matching the group's real name prefix", "[param][find]") {
    vehicle.pid.p.set(1.5f);
    vehicle.pid.i.set(0.5f);
    VarType ptype = VarType::None;

    void* p = find("PID_P", vehicle_info, ptype);
    REQUIRE(p == &vehicle.pid.p);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(static_cast<ParamFloat*>(p)->get() == 1.5f);

    void* i = find("PID_I", vehicle_info, ptype);
    REQUIRE(i == &vehicle.pid.i);
}

TEST_CASE("find's GROUP-name-prefix check is case-SENSITIVE, unlike its own SCALAR branch or find_group's nested match - reproduces upstream's own real inconsistency", "[param][find]") {
    VarType ptype = VarType::None;
    // "pid_p" (lowercase prefix) must NOT match the group named "PID_" -
    // upstream's real find() uses strncmp (case-sensitive) for this
    // check, not strcasecmp.
    REQUIRE(find("pid_p", vehicle_info, ptype) == nullptr);
}

TEST_CASE("find returns nullptr for an unknown name", "[param][find]") {
    VarType ptype = VarType::None;
    REQUIRE(find("NOPE", vehicle_info, ptype) == nullptr);
}

TEST_CASE("find keeps scanning later entries after a group name prefix matches but the suffix isn't found inside it", "[param][find]") {
    VarType ptype = VarType::None;
    // "PID_Q" starts with "PID_" but "Q" isn't in sub_table - upstream's
    // own comment: keep looking, don't stop the whole search just
    // because a group's own prefix matched.
    REQUIRE(find("PID_Q", vehicle_info, ptype) == nullptr);
}
