#pragma once

// Port of AP_Param::type_size and AP_Param::scan (AP_Param.cpp). CPP-022,
// slice 4. See ADR-0013 for the AP_Param sub-effort's overall scoping.
//
// scan() is the shared primitive both load and save are built on: a
// linear walk of a StorageAccess area from just past the EEPROM_header,
// looking for a ParamHeader matching a target (type + key +
// group_element), stopping at the first sentinel it finds. This slice
// covers scan() itself, fully self-contained and testable against
// CPP-020's ParamStorage/StorageAccess and CPP-021's ParamHeader/
// sentinel encoding - it needs neither of CPP-022's still-unported
// pieces (AP_Param::find's top-level table walk, or find_var_info's
// by-pointer-identity self-discovery).
//
// sentinel_offset CACHING NOT REPRODUCED: upstream caches the last-found
// sentinel offset in a static AP_Param::sentinel_offset member, updated
// by scan()/write_sentinel() and read directly by other callers (e.g.
// storage_used()) without re-scanning. This port has no hidden global
// state (matching every other module here) and doesn't yet port those
// other direct-cache-reader callers - scan() itself does the identical
// linear-scan algorithm upstream's own scan() does regardless of the
// cache (the cache is upstream's OWN optimization for OTHER call sites,
// not part of scan()'s own correctness), so nothing about scan()'s
// actual behavior is simplified. Callers that want the sentinel offset
// get it as an explicit out-parameter instead of a hidden static.
//
// LITERAL SAFETY: no bare ambiguous double literals - everything here is
// integer/byte arithmetic.

#include <cstdint>

#include <fwcpp/param/param.hpp> // EepromHeader, ParamHeader, VarType, get_key, is_sentinel
#include <fwcpp/param/storage.hpp> // storage::StorageAccess

namespace fwcpp::param {

// Byte size of a value of the given type, matching upstream's own
// switch exactly (including returning 0 for None/Group, which have no
// value payload of their own).
[[nodiscard]] inline constexpr std::uint8_t type_size(VarType type) {
    switch (type) {
        case VarType::None:
        case VarType::Group:
            return 0;
        case VarType::Int8:
            return 1;
        case VarType::Int16:
            return 2;
        case VarType::Int32:
            return 4;
        case VarType::Float:
            return 4;
        case VarType::Vector3f:
            return 3 * 4;
    }
    return 0; // unreachable for a valid VarType - matches upstream's own fallback
}

// Scans `storage` (a StorageAccess over the Param storage area) starting
// just past the EEPROM_header, looking for a ParamHeader matching
// `target`'s type/key/group_element.
//
// Returns true and sets `found_offset` to the matching entry's own
// offset if found. Returns false if not found: `found_offset` is set to
// the sentinel's offset (where a new entry should be written) and
// `sentinel_offset` mirrors it, UNLESS storage is exhausted before a
// sentinel is ever found, in which case `found_offset` is 0xFFFF -
// matches upstream's own three-way outcome exactly.
[[nodiscard]] inline bool scan(const storage::StorageAccess& storage, const ParamHeader& target, std::uint16_t& found_offset, std::uint16_t& sentinel_offset) {
    std::uint16_t ofs = sizeof(EepromHeader);
    while (ofs < storage.size()) {
        ParamHeader phdr{};
        if (!storage.read_block(&phdr, ofs, sizeof(phdr))) {
            break;
        }
        if (phdr.type == target.type && get_key(phdr) == get_key(target) && phdr.group_element == target.group_element) {
            found_offset = ofs;
            return true;
        }
        if (is_sentinel(phdr)) {
            found_offset = ofs;
            sentinel_offset = ofs;
            return false;
        }
        ofs = static_cast<std::uint16_t>(ofs + type_size(static_cast<VarType>(phdr.type)) + sizeof(phdr));
    }
    found_offset = 0xFFFF;
    return false;
}

} // namespace fwcpp::param
