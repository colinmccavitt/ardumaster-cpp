// Tests for fwcpp::param::GroupInfo/Info, group_id, get_base, and
// adjust_group_offset (CPP-022 slices 1-2).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/group_info.hpp>

#include <cstddef>

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

TEST_CASE("get_base without kFlagPointer returns info.ptr's own address directly", "[param][get_base]") {
    int dummy_object = 0;
    Info info{};
    info.ptr = &dummy_object;
    info.flags = 0;

    std::ptrdiff_t base = 0;
    REQUIRE(get_base(info, base));
    REQUIRE(base == reinterpret_cast<std::ptrdiff_t>(&dummy_object));
}

TEST_CASE("get_base with kFlagPointer dereferences info.ptr as a pointer-to-the-real-object", "[param][get_base]") {
    int real_object = 42;
    void* stored_ptr = &real_object; // this is what info.ptr "points at": a slot holding the real address

    Info info{};
    info.ptr = &stored_ptr;
    info.flags = kFlagPointer;

    std::ptrdiff_t base = 0;
    REQUIRE(get_base(info, base));
    REQUIRE(base == reinterpret_cast<std::ptrdiff_t>(&real_object));
}

TEST_CASE("get_base with kFlagPointer fails when the target isn't allocated (nullptr) yet", "[param][get_base]") {
    void* stored_ptr = nullptr;
    Info info{};
    info.ptr = &stored_ptr;
    info.flags = kFlagPointer;

    std::ptrdiff_t base = 0;
    REQUIRE_FALSE(get_base(info, base));
}

TEST_CASE("adjust_group_offset with kFlagNestedOffset adds group_info.offset to new_offset", "[param][adjust_group_offset]") {
    GroupInfo group_info{};
    group_info.flags = kFlagNestedOffset;
    group_info.offset = 16;

    std::ptrdiff_t new_offset = 4;
    REQUIRE(adjust_group_offset(0 /* base unused for this flag */, group_info, new_offset));
    REQUIRE(new_offset == 20);
}

TEST_CASE("adjust_group_offset with neither flag leaves new_offset untouched", "[param][adjust_group_offset]") {
    GroupInfo group_info{}; // flags = 0
    std::ptrdiff_t new_offset = 7;
    REQUIRE(adjust_group_offset(100, group_info, new_offset));
    REQUIRE(new_offset == 7);
}

TEST_CASE("adjust_group_offset with kFlagPointer resolves through an allocated sub-object pointer", "[param][adjust_group_offset]") {
    struct Container {
        void* sub_object_ptr;
    };
    int real_sub_object = 0;
    Container container{&real_sub_object};

    const std::ptrdiff_t base = reinterpret_cast<std::ptrdiff_t>(&container);

    GroupInfo group_info{};
    group_info.flags = kFlagPointer;
    group_info.offset = offsetof(Container, sub_object_ptr);

    std::ptrdiff_t new_offset = 0;
    REQUIRE(adjust_group_offset(base, group_info, new_offset));
    // new_offset should now be the delta from base to the real sub-object
    REQUIRE(base + new_offset == reinterpret_cast<std::ptrdiff_t>(&real_sub_object));
}

TEST_CASE("adjust_group_offset with kFlagPointer fails when the sub-object isn't allocated yet", "[param][adjust_group_offset]") {
    struct Container {
        void* sub_object_ptr;
    };
    Container container{nullptr};
    const std::ptrdiff_t base = reinterpret_cast<std::ptrdiff_t>(&container);

    GroupInfo group_info{};
    group_info.flags = kFlagPointer;
    group_info.offset = offsetof(Container, sub_object_ptr);

    std::ptrdiff_t new_offset = 0;
    REQUIRE_FALSE(adjust_group_offset(base, group_info, new_offset));
}
