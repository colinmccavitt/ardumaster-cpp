// Tests for fwcpp::storage::RawStorage/StorageAccess (CPP-020).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/storage.hpp>

#include <cstdio>
#include <cstring>

using namespace fwcpp::storage;

TEST_CASE("RawStorage starts zero-filled and is bounds-checked", "[storage][raw]") {
    RawStorage raw;
    REQUIRE(raw.size() == kStorageSize);

    std::uint8_t buf[4] = {1, 2, 3, 4};
    REQUIRE(raw.read_block(buf, 0, sizeof(buf)));
    REQUIRE(buf[0] == 0);
    REQUIRE(buf[3] == 0);

    // out of range: neither read nor write should succeed or crash
    REQUIRE_FALSE(raw.read_block(buf, static_cast<std::uint16_t>(kStorageSize - 1), 4));
    REQUIRE_FALSE(raw.write_block(static_cast<std::uint16_t>(kStorageSize - 1), buf, 4));
}

TEST_CASE("RawStorage write_block then read_block round-trips", "[storage][raw]") {
    RawStorage raw;
    const std::uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    REQUIRE(raw.write_block(100, data, sizeof(data)));

    std::uint8_t out[4] = {};
    REQUIRE(raw.read_block(out, 100, sizeof(out)));
    REQUIRE(std::memcmp(data, out, sizeof(data)) == 0);
}

TEST_CASE("RawStorage::erase fills with 0xFF, matching erased-flash semantics", "[storage][raw]") {
    RawStorage raw;
    raw.erase();
    std::uint8_t buf[8] = {};
    REQUIRE(raw.read_block(buf, 0, sizeof(buf)));
    for (std::uint8_t b : buf) {
        REQUIRE(b == 0xFF);
    }
}

TEST_CASE("RawStorage save_to_file then load_from_file round-trips", "[storage][raw][file]") {
    const char* path = "test_storage_roundtrip.bin";
    RawStorage raw;
    const std::uint8_t data[] = {1, 2, 3, 4, 5};
    REQUIRE(raw.write_block(42, data, sizeof(data)));
    REQUIRE(raw.save_to_file(path));

    RawStorage loaded;
    REQUIRE(loaded.load_from_file(path));
    std::uint8_t out[5] = {};
    REQUIRE(loaded.read_block(out, 42, sizeof(out)));
    REQUIRE(std::memcmp(data, out, sizeof(data)) == 0);

    std::remove(path);
}

TEST_CASE("RawStorage load_from_file fails cleanly for a missing file", "[storage][raw][file]") {
    RawStorage raw;
    REQUIRE_FALSE(raw.load_from_file("this_file_should_not_exist_12345.bin"));
}

// StorageAccess sizes, computed by hand from kLayout's own entries (see
// storage.hpp) - each StorageType's areas summed in table order, cross-
// checked against upstream StorageManager.cpp's cumulative #if
// STORAGE_NUM_AREAS >= N blocks for HAL_STORAGE_SIZE=16384 (Plane/SITL).
TEST_CASE("StorageAccess::size sums all areas of its own type", "[storage][access]") {
    RawStorage raw;
    REQUIRE(StorageAccess(raw, StorageType::Param).size() == 1280 + 1280 + 1280);      // 3840
    REQUIRE(StorageAccess(raw, StorageType::Mission).size() == 2506 + 2132 + 5204);    // 9842
    REQUIRE(StorageAccess(raw, StorageType::Rally).size() == 150 + 300 + 300);         // 750
    REQUIRE(StorageAccess(raw, StorageType::Fence).size() == 160 + 256 + 256);         // 672
    REQUIRE(StorageAccess(raw, StorageType::Keys).size() == 64);
    REQUIRE(StorageAccess(raw, StorageType::BindInfo).size() == 56);
    REQUIRE(StorageAccess(raw, StorageType::CANDNA).size() == 1024);
    REQUIRE(StorageAccess(raw, StorageType::ParamBak).size() == 0); // not in this port's 15-area layout
}

TEST_CASE("StorageAccess read/write within a single area round-trips", "[storage][access]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    const std::uint32_t value = 0x12345678;
    params.write_uint32(10, value);
    REQUIRE(params.read_uint32(10) == value);
}

TEST_CASE("StorageAccess read/write splits across an area boundary transparently", "[storage][access]") {
    // Param's first area is [0,1280); its second logical area (physical
    // offset 4096) starts at logical address 1280. A block starting at
    // 1276 and 8 bytes long spans both.
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    std::uint8_t data[8];
    for (std::uint8_t i = 0; i < 8; ++i) {
        data[i] = i + 1;
    }
    REQUIRE(params.write_block(1276, data, sizeof(data)));

    std::uint8_t out[8] = {};
    REQUIRE(params.read_block(out, 1276, sizeof(out)));
    REQUIRE(std::memcmp(data, out, sizeof(data)) == 0);

    // Confirm it actually landed in the physically separate area: bytes
    // 4 and 5 of `data` (logical addr 1280, 1281) should be readable
    // directly from RawStorage at PHYSICAL offset 4096 (the second
    // Param area's own offset in kLayout), not sequentially after 1280.
    std::uint8_t direct[4] = {};
    REQUIRE(raw.read_block(direct, 4096, sizeof(direct)));
    REQUIRE(direct[0] == data[4]);
    REQUIRE(direct[1] == data[5]);
}

TEST_CASE("StorageAccess for different types do not overlap", "[storage][access]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);
    StorageAccess mission(raw, StorageType::Mission);

    params.write_uint32(0, 0xAAAAAAAA);
    mission.write_uint32(0, 0xBBBBBBBB);

    REQUIRE(params.read_uint32(0) == 0xAAAAAAAA);
    REQUIRE(mission.read_uint32(0) == 0xBBBBBBBB);
}

TEST_CASE("StorageAccess read/write fails past the end of its own type's total size", "[storage][access]") {
    RawStorage raw;
    StorageAccess keys(raw, StorageType::Keys); // only 64 bytes total
    std::uint8_t buf[8] = {};
    REQUIRE_FALSE(keys.read_block(buf, 60, 8)); // 60+8 > 64
    REQUIRE_FALSE(keys.write_block(60, buf, 8));
}

TEST_CASE("StorageAccess byte/uint16/uint32/float helpers round-trip", "[storage][access]") {
    RawStorage raw;
    StorageAccess params(raw, StorageType::Param);

    params.write_byte(0, 0x7F);
    REQUIRE(params.read_byte(0) == 0x7F);

    params.write_uint16(2, 0xBEEF);
    REQUIRE(params.read_uint16(2) == 0xBEEF);

    params.write_float(8, 3.5f);
    REQUIRE(params.read_float(8) == 3.5f);
}
