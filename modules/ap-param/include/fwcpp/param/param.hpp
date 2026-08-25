#pragma once

// Port of AP_Param/AP_Param.h + AP_Param.cpp's byte-format layer:
// EEPROM_header, Param_header, the 9-bit key split, sentinel encoding,
// and the ParamFloat/ParamInt8/ParamInt16/ParamInt32 typed value
// wrappers. CPP-021. See ADR-0013 for the full scoping rationale (full
// upstream byte-compatible format, layered on CPP-020's ParamStorage).
//
// SLICE BOUNDARY: this is the storage-INDEPENDENT half of AP_Param - the
// header/key/sentinel encoding is pure bit-packing with no dependency on
// WHERE a value lives, and the typed wrappers here only hold an in-memory
// value (get/set), not yet knowing their own EEPROM address. Upstream's
// AP_ParamTBase adds set_default/set_and_default/set_and_notify/
// set_and_save/set_and_save_ifchanged - all of which need the tree/
// storage machinery (an object's Info/GroupInfo entry, to know where to
// write) that doesn't exist until CPP-022's GroupInfo/Info tree lands.
// Deliberately not stubbed here with a "TODO: no-op save()" - that would
// silently misrepresent what these types can do today. get()/set() are
// the honest subset: an in-memory value, persistable once CPP-022 wires
// it to storage.
//
// VarType matches upstream's ap_var_type enum values exactly (they're
// part of the on-storage format via Param_header.type, not arbitrary).
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant
// here is an integer (magic bytes, revision, bit-widths) or bit-cast of
// already-typed values.

#include <bit>
#include <cstdint>

namespace fwcpp::param {

// Matches upstream ap_var_type exactly - values are part of the
// Param_header.type on-storage encoding.
enum class VarType : std::uint8_t {
    None = 0,
    Int8 = 1,
    Int16 = 2,
    Int32 = 3,
    Float = 4,
    Vector3f = 5,
    Group = 6,
};

// 4 bytes, at the start of the storage area for AP_Param. magic[2]
// identifies this as ArduPilot parameter storage ("AP"); revision lets
// AP_Param detect and re-initialize storage written by an incompatible
// older/newer format (CPP-023's conversion machinery is the OTHER half
// of that story - migrating old-revision data instead of discarding it -
// deferred, not dropped).
struct EepromHeader {
    std::uint8_t magic[2] = {0, 0};
    std::uint8_t revision = 0;
    std::uint8_t spare = 0;
};
static_assert(sizeof(EepromHeader) == 4, "EepromHeader must match upstream's 4-byte layout");

inline constexpr std::uint8_t kEepromMagic0 = 0x50;
inline constexpr std::uint8_t kEepromMagic1 = 0x41; // "AP"
inline constexpr std::uint8_t kEepromRevision = 6;  // upstream k_EEPROM_revision

[[nodiscard]] inline constexpr EepromHeader make_eeprom_header() {
    return EepromHeader{kEepromMagic0, kEepromMagic1, kEepromRevision, 0};
}

[[nodiscard]] inline constexpr bool eeprom_header_valid(const EepromHeader& hdr) {
    return hdr.magic[0] == kEepromMagic0 && hdr.magic[1] == kEepromMagic1 && hdr.revision == kEepromRevision;
}

// Prepended to every variable stored under a ParamHeader-tagged region.
// key: the k_param enum value identifying which top-level parameter this
// is (a 9-bit value split across key_low/key_high - see the file banner
// quote of upstream's own comment: "to get 9 bits for key we needed to
// split it into two parts to keep binary compatibility"). group_element:
// zero for top-level parameters; for a parameter nested inside an
// object, three 6-bit fields packed together, one per nesting level
// (CPP-022's group_id encoding). type: the VarType of the variable.
struct ParamHeader {
    std::uint32_t key_low : 8 = 0;
    std::uint32_t type : 5 = 0;
    std::uint32_t key_high : 1 = 0;
    std::uint32_t group_element : 18 = 0;
};
static_assert(sizeof(ParamHeader) == 4, "ParamHeader must match upstream's 4-byte bitfield layout");

// Sentinel: marks the end of the written parameter list during a linear
// scan. Sentinel values, not sentinel object - matches upstream's own
// three independent sentinel constants (_sentinel_key/_sentinel_type/
// _sentinel_group), each checked separately by is_sentinel below.
inline constexpr std::uint16_t kSentinelKey = 0x1FF;
inline constexpr std::uint8_t kSentinelType = 0x1F;
inline constexpr std::uint8_t kSentinelGroup = 0xFF;

[[nodiscard]] inline constexpr std::uint16_t get_key(const ParamHeader& phdr) {
    return (static_cast<std::uint16_t>(phdr.key_high) << 8) | phdr.key_low;
}

inline constexpr void set_key(ParamHeader& phdr, std::uint16_t key) {
    phdr.key_low = key & 0xFF;
    phdr.key_high = (key >> 8) & 0x1;
}

[[nodiscard]] inline constexpr ParamHeader make_sentinel_header() {
    ParamHeader phdr{};
    phdr.type = kSentinelType;
    set_key(phdr, kSentinelKey);
    phdr.group_element = kSentinelGroup;
    return phdr;
}

// True if phdr marks the end of the parameter list, OR looks like the
// erased/blank fill pattern storage reads back as after a power loss
// mid-write (all-0xFF from erased flash, all-0x00 from a fresh/zeroed
// buffer) - matches upstream's own `||`, not `&&`, on type/key (its own
// comment: "this makes us more robust to power off while adding a
// variable"). std::bit_cast, not upstream's raw `*(uint32_t*)&phdr`
// reinterpret - same ADR-0012 no-unsafe-reinterpretation treatment
// already applied to Float16's union-based type punning.
[[nodiscard]] inline constexpr bool is_sentinel(const ParamHeader& phdr) {
    if (phdr.type == kSentinelType || get_key(phdr) == kSentinelKey) {
        return true;
    }
    const std::uint32_t v = std::bit_cast<std::uint32_t>(phdr);
    return v == 0 || v == 0xFFFFFFFFU;
}

// Typed in-memory value wrapper - see file banner for the CPP-021/
// CPP-022 scope split (get/set only; no persistence yet).
template <typename T>
class ParamValue {
public:
    ParamValue() = default;
    explicit ParamValue(const T& v) : value_(v) {}

    [[nodiscard]] const T& get() const { return value_; }
    void set(const T& v) { value_ = v; }

    // Matches upstream AP_ParamT<T,PT>'s implicit `operator const T&()` -
    // same documented tradeoff upstream accepts (silent narrowing in an
    // expression like `int16_t v = cond ? int16_param : (int8_t)0`);
    // `.get()` is available when that matters.
    operator const T&() const { return value_; }

private:
    T value_{};
};

// AP_Float specifically forbids the implicit-int/double-conversion
// footgun upstream's own comment describes (a templated conversion
// operator so further implicit conversions require an exact match) -
// reproduced with the same technique, not simplified away, since it's
// the actual reason AP_Float doesn't use the generic AP_ParamT shape
// upstream either (see AP_Param.h's own AP_ParamT<float,...> full
// specialization).
class ParamFloat {
public:
    ParamFloat() = default;
    explicit ParamFloat(float v) : value_(v) {}

    [[nodiscard]] const float& get() const { return value_; }
    void set(float v) { value_ = v; }

    template <bool = true>
    operator const float&() const {
        return value_;
    }

    explicit operator int() const { return static_cast<int>(value_); }
    explicit operator double() const { return static_cast<double>(value_); }

private:
    float value_ = 0.0f;
};

using ParamInt8 = ParamValue<std::int8_t>;
using ParamInt16 = ParamValue<std::int16_t>;
using ParamInt32 = ParamValue<std::int32_t>;

} // namespace fwcpp::param
