// Tests for fwcpp::param's byte-format layer (CPP-021): EepromHeader,
// ParamHeader, key/sentinel encoding, and the typed value wrappers.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/param/param.hpp>

#include <bit>
#include <cstdint>

using namespace fwcpp::param;

TEST_CASE("EepromHeader and ParamHeader are exactly 4 bytes, matching upstream's static_assert", "[param][format]") {
    STATIC_REQUIRE(sizeof(EepromHeader) == 4);
    STATIC_REQUIRE(sizeof(ParamHeader) == 4);
}

TEST_CASE("make_eeprom_header produces upstream's magic bytes and revision", "[param][format]") {
    EepromHeader hdr = make_eeprom_header();
    REQUIRE(hdr.magic[0] == 0x50);
    REQUIRE(hdr.magic[1] == 0x41); // "AP"
    REQUIRE(hdr.revision == 6);
}

TEST_CASE("eeprom_header_valid accepts a freshly-made header and rejects wrong magic/revision", "[param][format]") {
    REQUIRE(eeprom_header_valid(make_eeprom_header()));

    EepromHeader bad_magic = make_eeprom_header();
    bad_magic.magic[0] = 0x00;
    REQUIRE_FALSE(eeprom_header_valid(bad_magic));

    EepromHeader bad_revision = make_eeprom_header();
    bad_revision.revision = 5;
    REQUIRE_FALSE(eeprom_header_valid(bad_revision));
}

TEST_CASE("get_key/set_key round-trip across the full 9-bit key range", "[param][key]") {
    for (std::uint16_t key : {0, 1, 255, 256, 257, 510, 511}) {
        ParamHeader phdr{};
        set_key(phdr, key);
        REQUIRE(get_key(phdr) == key);
    }
}

TEST_CASE("set_key splits the 9-bit key across key_low (bits 0-7) and key_high (bit 8), matching upstream exactly", "[param][key]") {
    ParamHeader phdr{};
    set_key(phdr, 0x1AB); // binary 1_1010_1011 -> key_low=0xAB, key_high=1
    REQUIRE(phdr.key_low == 0xAB);
    REQUIRE(phdr.key_high == 1);

    ParamHeader phdr2{};
    set_key(phdr2, 0x0AB); // key_high should be 0
    REQUIRE(phdr2.key_low == 0xAB);
    REQUIRE(phdr2.key_high == 0);
}

TEST_CASE("make_sentinel_header is recognized by is_sentinel", "[param][sentinel]") {
    REQUIRE(is_sentinel(make_sentinel_header()));
}

TEST_CASE("is_sentinel recognizes the erased-flash (0xFF) and zeroed (0x00) fill patterns", "[param][sentinel]") {
    // std::bit_cast the other way: build via known bit pattern rather than
    // reaching into the bitfields directly, matching how a real erased
    // flash region would actually look byte-for-byte.
    ParamHeader all_ff = std::bit_cast<ParamHeader>(static_cast<std::uint32_t>(0xFFFFFFFFU));
    REQUIRE(is_sentinel(all_ff));

    ParamHeader all_zero = std::bit_cast<ParamHeader>(static_cast<std::uint32_t>(0));
    REQUIRE(is_sentinel(all_zero));
}

TEST_CASE("is_sentinel is false for an ordinary, real parameter header", "[param][sentinel]") {
    ParamHeader phdr{};
    set_key(phdr, 5);
    phdr.type = static_cast<std::uint32_t>(VarType::Float);
    phdr.group_element = 0;
    REQUIRE_FALSE(is_sentinel(phdr));
}

TEST_CASE("is_sentinel is true if EITHER type or key matches its own sentinel value, not just both (upstream's own || choice)", "[param][sentinel]") {
    // Sentinel type but a non-sentinel key - upstream deliberately uses
    // || here for power-loss robustness (see is_sentinel's own comment).
    ParamHeader partial{};
    set_key(partial, 5); // ordinary key
    partial.type = kSentinelType; // sentinel type
    REQUIRE(is_sentinel(partial));
}

// Verbatim reproduction of upstream's own Param_header declaration
// (AP_Param.h lines 654-660), kept local to this test rather than
// #included from upstream (which would drag in AP_HAL/StorageManager
// and the rest of AP_Param's own dependency chain just to check one
// struct's layout). Both this port and upstream are compiled by the same
// GCC on the same platform, so identical bitfield declarations pack
// identically - this proves that directly, rather than assuming it.
namespace upstream_reference {
struct Param_header {
    std::uint32_t key_low : 8;
    std::uint32_t type : 5;
    std::uint32_t key_high : 1;
    std::uint32_t group_element : 18;
};
} // namespace upstream_reference

TEST_CASE("ParamHeader packs identically to upstream's own Param_header declaration", "[param][format][crosscheck]") {
    STATIC_REQUIRE(sizeof(ParamHeader) == sizeof(upstream_reference::Param_header));

    // Fill every field with a distinct, easily-recognized value and
    // confirm both structs, viewed as raw uint32_t, produce the exact
    // same bit pattern - a direct proof of identical field ordering and
    // bit-widths, not just equal overall size.
    ParamHeader ours{};
    ours.key_low = 0xAB;
    ours.type = 0x15; // 5 bits: max 0x1F
    ours.key_high = 1;
    ours.group_element = 0x2AAAA; // 18 bits: max 0x3FFFF

    upstream_reference::Param_header theirs{};
    theirs.key_low = 0xAB;
    theirs.type = 0x15;
    theirs.key_high = 1;
    theirs.group_element = 0x2AAAA;

    REQUIRE(std::bit_cast<std::uint32_t>(ours) == std::bit_cast<std::uint32_t>(theirs));
}

TEST_CASE("ParamValue<int32_t> get/set and implicit conversion", "[param][value]") {
    ParamInt32 p(42);
    REQUIRE(p.get() == 42);
    p.set(-7);
    REQUIRE(p.get() == -7);
    const std::int32_t& ref = p; // implicit operator const T&()
    REQUIRE(ref == -7);
}

TEST_CASE("ParamValue<int8_t>/int16_t default-construct to zero", "[param][value]") {
    ParamInt8 a;
    ParamInt16 b;
    REQUIRE(a.get() == 0);
    REQUIRE(b.get() == 0);
}

TEST_CASE("ParamFloat get/set and explicit int/double conversions", "[param][value]") {
    ParamFloat f(3.5f);
    REQUIRE(f.get() == 3.5f);
    f.set(-2.25f);
    REQUIRE(f.get() == -2.25f);

    const float& ref = f; // implicit operator const float&()
    REQUIRE(ref == -2.25f);

    REQUIRE(static_cast<int>(f) == -2); // explicit, truncating toward zero
    REQUIRE(static_cast<double>(f) == Catch::Approx(-2.25));
}
