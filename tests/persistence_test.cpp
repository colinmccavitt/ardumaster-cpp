// Tests for fwcpp::param::type_size/scan (CPP-022 slice 4).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/persistence.hpp>

using namespace fwcpp::param;
using fwcpp::storage::RawStorage;
using fwcpp::storage::StorageAccess;
using fwcpp::storage::StorageType;

TEST_CASE("type_size matches upstream's byte sizes for every VarType", "[param][type_size]") {
    REQUIRE(type_size(VarType::None) == 0);
    REQUIRE(type_size(VarType::Group) == 0);
    REQUIRE(type_size(VarType::Int8) == 1);
    REQUIRE(type_size(VarType::Int16) == 2);
    REQUIRE(type_size(VarType::Int32) == 4);
    REQUIRE(type_size(VarType::Float) == 4);
    REQUIRE(type_size(VarType::Vector3f) == 12);
}

namespace {
ParamHeader make_header(std::uint16_t key, VarType type, std::uint32_t group_element = 0) {
    ParamHeader h{};
    set_key(h, key);
    h.type = static_cast<std::uint32_t>(type);
    h.group_element = group_element;
    return h;
}
} // namespace

TEST_CASE("scan on empty (zero-filled) storage immediately reports the sentinel right after the EEPROM header", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader target = make_header(5, VarType::Float);
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE_FALSE(scan(params, target, found_offset, sentinel_offset));
    REQUIRE(found_offset == sizeof(EepromHeader));
    REQUIRE(sentinel_offset == sizeof(EepromHeader));
}

TEST_CASE("scan finds a single written entry matching key/type/group_element", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader entry = make_header(7, VarType::Int32);
    const std::uint16_t entry_ofs = sizeof(EepromHeader);
    std::int32_t value = 12345;
    REQUIRE(params.write_block(entry_ofs, &entry, sizeof(entry)));
    REQUIRE(params.write_block(static_cast<std::uint16_t>(entry_ofs + sizeof(entry)), &value, sizeof(value)));
    // leave the rest zero-filled, which is_sentinel treats as the sentinel

    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE(scan(params, entry, found_offset, sentinel_offset));
    REQUIRE(found_offset == entry_ofs);
}

TEST_CASE("scan skips past a non-matching entry using its type_size to find the sentinel", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader entry = make_header(7, VarType::Int32); // 4-byte value
    const std::uint16_t entry_ofs = sizeof(EepromHeader);
    std::int32_t value = 1;
    REQUIRE(params.write_block(entry_ofs, &entry, sizeof(entry)));
    REQUIRE(params.write_block(static_cast<std::uint16_t>(entry_ofs + sizeof(entry)), &value, sizeof(value)));
    // rest is zero (sentinel)

    ParamHeader different_target = make_header(8, VarType::Int32); // different key, not present
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE_FALSE(scan(params, different_target, found_offset, sentinel_offset));
    // should have advanced past entry (header+value = 4+4=8 bytes) to find the sentinel
    REQUIRE(found_offset == entry_ofs + sizeof(ParamHeader) + 4);
}

TEST_CASE("scan finds the SECOND of two written entries, skipping the first correctly", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    std::uint16_t ofs = sizeof(EepromHeader);

    ParamHeader entry1 = make_header(1, VarType::Int8); // 1-byte value
    std::int8_t v1 = 5;
    REQUIRE(params.write_block(ofs, &entry1, sizeof(entry1)));
    REQUIRE(params.write_block(static_cast<std::uint16_t>(ofs + sizeof(entry1)), &v1, sizeof(v1)));
    ofs = static_cast<std::uint16_t>(ofs + sizeof(entry1) + sizeof(v1));

    ParamHeader entry2 = make_header(2, VarType::Float); // 4-byte value
    float v2 = 3.25f;
    REQUIRE(params.write_block(ofs, &entry2, sizeof(entry2)));
    REQUIRE(params.write_block(static_cast<std::uint16_t>(ofs + sizeof(entry2)), &v2, sizeof(v2)));
    const std::uint16_t entry2_ofs = ofs;

    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE(scan(params, entry2, found_offset, sentinel_offset));
    REQUIRE(found_offset == entry2_ofs);
}

TEST_CASE("scan distinguishes entries by group_element even with the same key/type", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    const std::uint16_t ofs = sizeof(EepromHeader);
    ParamHeader entry = make_header(9, VarType::Int16, /*group_element=*/42);
    std::int16_t value = 100;
    REQUIRE(params.write_block(ofs, &entry, sizeof(entry)));
    REQUIRE(params.write_block(static_cast<std::uint16_t>(ofs + sizeof(entry)), &value, sizeof(value)));

    ParamHeader same_key_different_group = make_header(9, VarType::Int16, /*group_element=*/43);
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE_FALSE(scan(params, same_key_different_group, found_offset, sentinel_offset));
}

TEST_CASE("scan returns 0xFFFF if storage is exhausted before a sentinel is ever found", "[param][scan]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param); // 3840 bytes total (see CPP-020's kLayout)

    std::uint16_t ofs = sizeof(EepromHeader);
    std::uint16_t key = 1;
    while (ofs + sizeof(ParamHeader) + type_size(VarType::Int8) <= params.size()) {
        // Every entry uses a distinct key (mod 512, the 9-bit key space -
        // with ~767 entries needed to fill 3840 bytes, keys necessarily
        // wrap, but VarType::Int8 vs. the eventual VarType::Float search
        // target still guards against an accidental match) so none
        // accidentally matches the eventual search target. key==0x1FF
        // (511) is skipped deliberately: that's kSentinelKey itself, and
        // set_key(entry, 511) would make is_sentinel(entry) true - which
        // is exactly correct port behavior (matching upstream's own
        // reserved key value) but would stop this test's scan early
        // instead of exhausting storage as intended.
        if (key != kSentinelKey) {
            ParamHeader entry = make_header(key, VarType::Int8);
            std::int8_t value = 1;
            REQUIRE(params.write_block(ofs, &entry, sizeof(entry)));
            REQUIRE(params.write_block(static_cast<std::uint16_t>(ofs + sizeof(entry)), &value, sizeof(value)));
            ofs = static_cast<std::uint16_t>(ofs + sizeof(entry) + sizeof(value));
        }
        ++key;
    }

    ParamHeader never_present = make_header(9999, VarType::Float);
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE_FALSE(scan(params, never_present, found_offset, sentinel_offset));
    REQUIRE(found_offset == 0xFFFF);
}
