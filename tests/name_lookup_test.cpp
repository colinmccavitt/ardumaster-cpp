// Tests for fwcpp::param::find_group (CPP-022 slice 3).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/name_lookup.hpp>
#include <fwcpp/param/param.hpp>

#include <cstddef>

using namespace fwcpp::param;

namespace {
// Mirrors a real upstream layout: a parent object with a nested
// sub-object (e.g. AC_AttitudeControl's _p_angle_roll, registered via
// AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", ...) - note the trailing
// underscore baked into the GROUP's own name string, not appended by
// the lookup code. Verified against real upstream usage
// (AC_AttitudeControl.cpp/ArduPlane/Parameters.cpp), not assumed.
struct Inner {
    ParamFloat a;
    ParamInt8 b;
};

struct Outer {
    Inner inner;
    ParamFloat top;
};

GroupInfo make_scalar(const char* name, std::ptrdiff_t offset, std::uint8_t idx, VarType type) {
    GroupInfo g{};
    g.name = name;
    g.offset = offset;
    g.idx = idx;
    g.type = static_cast<std::uint8_t>(type);
    return g;
}

GroupInfo make_group(const char* name, std::ptrdiff_t offset, std::uint8_t idx, const GroupInfo* nested) {
    GroupInfo g{};
    g.name = name;
    g.offset = offset;
    g.idx = idx;
    g.type = static_cast<std::uint8_t>(VarType::Group);
    g.flags = kFlagNestedOffset;
    g.group_info = nested;
    return g;
}

GroupInfo make_sentinel() {
    GroupInfo g{};
    g.type = static_cast<std::uint8_t>(VarType::None);
    return g;
}

GroupInfo inner_table[] = {
    make_scalar("A", offsetof(Inner, a), 0, VarType::Float),
    make_scalar("B", offsetof(Inner, b), 1, VarType::Int8),
    make_sentinel(),
};

// Trailing underscore baked into the name itself - see comment above.
GroupInfo outer_table[] = {
    make_group("INNER_", offsetof(Outer, inner), 0, inner_table),
    make_scalar("TOP", offsetof(Outer, top), 1, VarType::Float),
    make_sentinel(),
};
} // namespace

TEST_CASE("find_group finds a top-level scalar by exact name", "[param][find_group]") {
    Outer obj{};
    obj.top.set(9.5f);

    VarType ptype = VarType::None;
    void* p = find_group("TOP", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype);
    REQUIRE(p != nullptr);
    REQUIRE(ptype == VarType::Float);
    REQUIRE(static_cast<ParamFloat*>(p) == &obj.top);
    REQUIRE(static_cast<ParamFloat*>(p)->get() == 9.5f);
}

TEST_CASE("find_group leaf matching is case-insensitive", "[param][find_group]") {
    Outer obj{};
    VarType ptype = VarType::None;
    REQUIRE(find_group("top", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype) == &obj.top);
    REQUIRE(find_group("Top", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype) == &obj.top);
}

TEST_CASE("find_group recurses into a nested group using its prefix (with the trailing separator baked into the group's own name)", "[param][find_group]") {
    Outer obj{};
    obj.inner.a.set(1.5f);
    VarType ptype = VarType::None;

    void* p = find_group("INNER_A", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype);
    REQUIRE(p == &obj.inner.a);
    REQUIRE(ptype == VarType::Float);
}

TEST_CASE("find_group's nested group prefix match is also case-insensitive", "[param][find_group]") {
    Outer obj{};
    VarType ptype = VarType::None;
    void* p = find_group("inner_b", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype);
    REQUIRE(p == &obj.inner.b);
    REQUIRE(ptype == VarType::Int8);
}

TEST_CASE("find_group returns nullptr for an unknown name", "[param][find_group]") {
    Outer obj{};
    VarType ptype = VarType::None;
    REQUIRE(find_group("NOPE", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype) == nullptr);
}

TEST_CASE("find_group requires the group prefix to match from the start, not just appear as a substring", "[param][find_group]") {
    Outer obj{};
    VarType ptype = VarType::None;
    // "XINNER_A" does not start with "INNER_" - must not match.
    REQUIRE(find_group("XINNER_A", reinterpret_cast<std::ptrdiff_t>(&obj), 0, outer_table, ptype) == nullptr);
}

TEST_CASE("find_group resolves addresses relative to different Outer instances independently", "[param][find_group]") {
    Outer obj1{};
    Outer obj2{};
    obj1.top.set(1.0f);
    obj2.top.set(2.0f);

    VarType ptype = VarType::None;
    void* p1 = find_group("TOP", reinterpret_cast<std::ptrdiff_t>(&obj1), 0, outer_table, ptype);
    void* p2 = find_group("TOP", reinterpret_cast<std::ptrdiff_t>(&obj2), 0, outer_table, ptype);

    REQUIRE(static_cast<ParamFloat*>(p1)->get() == 1.0f);
    REQUIRE(static_cast<ParamFloat*>(p2)->get() == 2.0f);
    REQUIRE(p1 != p2);
}
