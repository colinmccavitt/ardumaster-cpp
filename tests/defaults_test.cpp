// Tests for fwcpp::param::set_value/get_default_value/setup_object_defaults
// (CPP-022 slice 6).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/defaults.hpp>

#include <cstddef>

using namespace fwcpp::param;

namespace {
struct Gains {
    ParamFloat p;
    ParamFloat i;
    ParamInt8 enable;
    ParamInt32 count;
};

GroupInfo make_scalar(const char* name, std::ptrdiff_t offset, std::uint8_t idx, VarType type, float def_value) {
    GroupInfo g{};
    g.name = name;
    g.offset = offset;
    g.idx = idx;
    g.type = static_cast<std::uint8_t>(type);
    g.def_value = def_value;
    return g;
}

GroupInfo make_sentinel() {
    GroupInfo g{};
    g.type = static_cast<std::uint8_t>(VarType::None);
    return g;
}

GroupInfo gains_table[] = {
    make_scalar("P", offsetof(Gains, p), 0, VarType::Float, 1.5f),
    make_scalar("I", offsetof(Gains, i), 1, VarType::Float, 0.25f),
    make_scalar("ENABLE", offsetof(Gains, enable), 2, VarType::Int8, 1.0f),
    make_scalar("COUNT", offsetof(Gains, count), 3, VarType::Int32, 42.0f),
    make_sentinel(),
};
} // namespace

TEST_CASE("set_value dispatches to the correct concrete type by VarType", "[param][set_value]") {
    ParamFloat f;
    set_value(VarType::Float, &f, 7.5f);
    REQUIRE(f.get() == 7.5f);

    ParamInt8 i8;
    set_value(VarType::Int8, &i8, 100.0f);
    REQUIRE(i8.get() == 100);

    ParamInt16 i16;
    set_value(VarType::Int16, &i16, -500.0f);
    REQUIRE(i16.get() == -500);

    ParamInt32 i32;
    set_value(VarType::Int32, &i32, 70000.0f);
    REQUIRE(i32.get() == 70000);
}

TEST_CASE("set_value with VarType::None or VarType::Group is a documented no-op", "[param][set_value]") {
    ParamFloat f(9.0f);
    set_value(VarType::None, &f, 1.0f);
    REQUIRE(f.get() == 9.0f); // untouched
    set_value(VarType::Group, &f, 1.0f);
    REQUIRE(f.get() == 9.0f); // untouched
}

TEST_CASE("get_default_value returns the plain def_value when kFlagDefaultPointer is not set", "[param][get_default_value]") {
    GroupInfo info = make_scalar("X", 0, 0, VarType::Float, 3.75f);
    REQUIRE(get_default_value(nullptr, info) == 3.75f);
}

TEST_CASE("get_default_value reads from a sibling field when kFlagDefaultPointer is set", "[param][get_default_value]") {
    struct WithSharedDefault {
        float shared_default = 6.5f;
        ParamFloat value;
    };
    WithSharedDefault obj;

    GroupInfo info{};
    info.flags = kFlagDefaultPointer;
    // def_value_offset: distance from the PARAM field's own address back
    // to the sibling holding the real default - matches upstream's own
    // `*((float*)((ptrdiff_t)vp - info.def_value_offset))`.
    info.def_value_offset = reinterpret_cast<std::ptrdiff_t>(&obj.value) - reinterpret_cast<std::ptrdiff_t>(&obj.shared_default);

    REQUIRE(get_default_value(&obj.value, info) == 6.5f);

    obj.shared_default = 9.25f; // the "default" can be a live, shared value
    REQUIRE(get_default_value(&obj.value, info) == 9.25f);
}

TEST_CASE("setup_object_defaults applies every scalar default in a flat table", "[param][setup_object_defaults]") {
    Gains gains{};
    setup_object_defaults(&gains, gains_table);

    REQUIRE(gains.p.get() == 1.5f);
    REQUIRE(gains.i.get() == 0.25f);
    REQUIRE(gains.enable.get() == 1);
    REQUIRE(gains.count.get() == 42);
}

TEST_CASE("setup_object_defaults overwrites whatever value was already there", "[param][setup_object_defaults]") {
    Gains gains{};
    gains.p.set(999.0f);
    gains.enable.set(-1);

    setup_object_defaults(&gains, gains_table);

    REQUIRE(gains.p.get() == 1.5f);
    REQUIRE(gains.enable.get() == 1);
}

TEST_CASE("setup_object_defaults on two separate objects sets each independently", "[param][setup_object_defaults]") {
    Gains a{};
    Gains b{};
    setup_object_defaults(&a, gains_table);
    setup_object_defaults(&b, gains_table);

    a.p.set(0.0f);
    REQUIRE(b.p.get() == 1.5f); // unaffected by mutating `a`
}
