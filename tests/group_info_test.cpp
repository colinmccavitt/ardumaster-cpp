// Tests for fwcpp::param::GroupInfo/Info and group_id (CPP-022 slice 1).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/group_info.hpp>

using namespace fwcpp::param;

namespace {
GroupInfo make_entry(std::uint8_t idx, std::uint16_t flags = 0) {
    GroupInfo entry{};
    entry.idx = idx;
    entry.flags = flags;
    return entry;
}
} // namespace

TEST_CASE("group_id shifts idx by the given amount and adds it to base", "[param][group_id]") {
    GroupInfo table[] = {make_entry(5)};
    // idx=5, shift=0 -> base + (5<<0) = base + 5
    REQUIRE(group_id(table, 0, 0, 0) == 5);
    // idx=5, shift=6 -> base + (5<<6) = base + 320
    REQUIRE(group_id(table, 0, 0, 6) == 320);
    // nonzero base is preserved
    REQUIRE(group_id(table, 100, 0, 0) == 105);
}

TEST_CASE("group_id substitutes 63 for idx=0 at a nonzero shift (upstream's own aliasing workaround)", "[param][group_id]") {
    GroupInfo table[] = {make_entry(0)};
    // shift=0: idx 0 is unambiguous (base+0 can't alias anything else at this level) - no substitution
    REQUIRE(group_id(table, 0, 0, 0) == 0);
    // shift=6: idx 0 shifted is still 0, indistinguishable from "not nested"
    // - substituted with 63 instead, matching upstream's documented fix.
    REQUIRE(group_id(table, 0, 0, 6) == (63U << 6));
    // base is still added on top of the substituted value
    REQUIRE(group_id(table, 1000, 0, 6) == 1000 + (63U << 6));
}

TEST_CASE("group_id does NOT substitute when kFlagNoShift is set, even for idx=0 at a nonzero shift", "[param][group_id]") {
    GroupInfo table[] = {make_entry(0, kFlagNoShift)};
    REQUIRE(group_id(table, 0, 0, 6) == 0); // no substitution: caller has asserted no aliasing risk
}

TEST_CASE("GroupInfo/Info's anonymous union selects the member the flags designate", "[param][group_info]") {
    GroupInfo with_default{};
    with_default.def_value = 3.5f;
    REQUIRE(with_default.def_value == 3.5f);

    GroupInfo child_table[] = {make_entry(0)};
    GroupInfo with_group{};
    with_group.group_info = child_table;
    with_group.flags = kFlagNestedOffset;
    REQUIRE(with_group.group_info == child_table);
}

TEST_CASE("GroupInfo/Info are POD-shaped: default construction zero-initializes every field", "[param][group_info]") {
    GroupInfo entry{};
    REQUIRE(entry.name == nullptr);
    REQUIRE(entry.offset == 0);
    REQUIRE(entry.flags == 0);
    REQUIRE(entry.idx == 0);
    REQUIRE(entry.type == 0);

    Info info{};
    REQUIRE(info.ptr == nullptr);
    REQUIRE(info.key == 0);
}
