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

TEST_CASE("load_raw fails on a never-saved header", "[param][load_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);
    ParamHeader phdr = make_header(1, VarType::Float);
    float out = 0.0f;
    REQUIRE_FALSE(load_raw(params, phdr, &out, sizeof(out)));
}

TEST_CASE("save_raw then load_raw round-trips a new entry", "[param][save_raw][load_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader phdr = make_header(3, VarType::Float);
    float value = 12.5f;
    REQUIRE(save_raw(params, phdr, &value, sizeof(value)));

    float out = 0.0f;
    REQUIRE(load_raw(params, phdr, &out, sizeof(out)));
    REQUIRE(out == 12.5f);
}

TEST_CASE("save_raw on an existing header updates the value in place, not appending a duplicate", "[param][save_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader phdr = make_header(4, VarType::Int32);
    std::int32_t v1 = 100;
    REQUIRE(save_raw(params, phdr, &v1, sizeof(v1)));

    std::uint16_t offset_after_first_save = 0;
    std::uint16_t sentinel_after_first_save = 0;
    REQUIRE(scan(params, phdr, offset_after_first_save, sentinel_after_first_save));

    std::int32_t v2 = 200;
    REQUIRE(save_raw(params, phdr, &v2, sizeof(v2)));

    std::uint16_t offset_after_second_save = 0;
    std::uint16_t sentinel_after_second_save = 0;
    REQUIRE(scan(params, phdr, offset_after_second_save, sentinel_after_second_save));

    // Same offset: proves this was an in-place update, not a second
    // entry appended after the first (which would also have left the
    // stale first entry findable at its original offset - it isn't,
    // because there's only ever one entry with this header).
    REQUIRE(offset_after_first_save == offset_after_second_save);

    std::int32_t out = 0;
    REQUIRE(load_raw(params, phdr, &out, sizeof(out)));
    REQUIRE(out == 200);
}

TEST_CASE("save_raw appends multiple distinct entries, each independently loadable", "[param][save_raw][load_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader h1 = make_header(10, VarType::Int8);
    ParamHeader h2 = make_header(11, VarType::Int16);
    ParamHeader h3 = make_header(12, VarType::Float);

    std::int8_t v1 = 42;
    std::int16_t v2 = -1000;
    float v3 = 3.14f;

    REQUIRE(save_raw(params, h1, &v1, sizeof(v1)));
    REQUIRE(save_raw(params, h2, &v2, sizeof(v2)));
    REQUIRE(save_raw(params, h3, &v3, sizeof(v3)));

    std::int8_t o1 = 0;
    std::int16_t o2 = 0;
    float o3 = 0.0f;
    REQUIRE(load_raw(params, h1, &o1, sizeof(o1)));
    REQUIRE(load_raw(params, h2, &o2, sizeof(o2)));
    REQUIRE(load_raw(params, h3, &o3, sizeof(o3)));

    REQUIRE(o1 == 42);
    REQUIRE(o2 == -1000);
    REQUIRE(o3 == 3.14f);
}

TEST_CASE("after save_raw, a fresh sentinel is written immediately past the new entry (the write ordering's actual observable effect)", "[param][save_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    ParamHeader phdr = make_header(1, VarType::Int32);
    std::int32_t value = 5;
    REQUIRE(save_raw(params, phdr, &value, sizeof(value)));

    // Scanning for a still-never-saved header should stop right after
    // the entry just written, not run further into (zero-filled, also
    // sentinel-shaped) storage beyond it.
    ParamHeader other = make_header(2, VarType::Int32);
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    REQUIRE_FALSE(scan(params, other, found_offset, sentinel_offset));
    REQUIRE(found_offset == sizeof(EepromHeader) + sizeof(ParamHeader) + sizeof(value));
}

TEST_CASE("save_raw fails once storage genuinely has no room left for a new entry plus its trailing sentinel", "[param][save_raw]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param); // 3840 bytes total

    // Fill storage with Int32 entries (4-byte header + 4-byte value = 8
    // bytes each) until save_raw itself reports failure - this exercises
    // the real "out of room" check, not a hand-computed boundary.
    std::uint16_t key = 1;
    bool ran_out = false;
    for (int i = 0; i < 1000 && !ran_out; ++i) {
        if (key == kSentinelKey) {
            ++key;
            continue;
        }
        ParamHeader phdr = make_header(key, VarType::Int32);
        std::int32_t value = i;
        if (!save_raw(params, phdr, &value, sizeof(value))) {
            ran_out = true;
        }
        ++key;
    }
    REQUIRE(ran_out);
}
