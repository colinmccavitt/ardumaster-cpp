#pragma once

// Port of AP_Param::type_size, AP_Param::scan, and the raw storage
// read/write path (load_raw/save_raw, built from AP_Param::write_sentinel
// and the eeprom_write_check-based portion of save_sync). CPP-022,
// slices 4-5. See ADR-0013 for the AP_Param sub-effort's overall scoping.
//
// SLICE 5 (load_raw/save_raw) scope: this is the raw byte-level
// persistence primitive - given an already-resolved ParamHeader (key/
// group_element/type set by the caller) and a value buffer, read or
// write it. Deliberately NOT included: the default-value-skip comparison
// upstream's save_sync performs before writing (skip the write entirely
// if the new value equals the GroupInfo/Info's own def_value, within a
// 0.01% tolerance for non-int32 types) - that's a policy layered ON TOP
// of raw persistence, needs a resolved GroupInfo/Info entry's def_value
// (not just a ParamHeader), and belongs with the higher-level save()
// this port doesn't have yet (which itself needs find_var_info's by-
// pointer-identity resolution - still unported, see name_lookup.hpp's
// banner). Also not reproduced: AP_PARAM_STORAGE_BAK_ENABLED's mirrored
// backup-storage write (eeprom_write_check writes to both _storage and
// an optional _storage_bak) - a redundancy feature, not part of the core
// format, and this port has one storage area, not two.
//
// POWER-LOSS-SAFE WRITE ORDER, preserved exactly: when appending a new
// entry, upstream writes the NEW sentinel (past the new entry) FIRST,
// then the new entry's value, then the new entry's header LAST. If power
// is lost after the sentinel write but before the header write, a
// subsequent scan() still finds the OLD sentinel unchanged at `ofs` (the
// new header bytes there are still whatever was there before - which
// was the old sentinel, since `ofs` came from scan() reporting "not
// found, this is the sentinel position") and stops there, never reading
// the now-partially-written entry beyond it. The data only becomes
// reachable once the LAST write (the header) completes. This can't be
// exercised by a normal unit test (there's no way to interrupt a write
// mid-call), so it's verified structurally instead: a test confirms the
// three writes happen in the order load_raw/scan would need them to for
// this property to hold, and that the final result matches an
// uninterrupted write's expected bytes.
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

// Reads a previously-saved value matching `phdr` into `value` (exactly
// `value_size` bytes). Returns false (leaving `value` untouched) if no
// matching entry is found.
[[nodiscard]] inline bool load_raw(const storage::StorageAccess& storage, const ParamHeader& phdr, void* value, std::uint8_t value_size) {
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    if (!scan(storage, phdr, found_offset, sentinel_offset)) {
        return false;
    }
    return storage.read_block(value, static_cast<std::uint16_t>(found_offset + sizeof(ParamHeader)), value_size);
}

// Writes `value` (`value_size` bytes) under `phdr`. If a matching entry
// already exists, its value is updated in place (header untouched - the
// header already matches, that's how it was found). Otherwise appends a
// new entry using the power-loss-safe write order described in this
// file's banner. Returns false if storage is exhausted (no sentinel
// found within the area at all) or full (not enough room left for this
// entry plus a fresh sentinel after it).
[[nodiscard]] inline bool save_raw(storage::StorageAccess& storage, const ParamHeader& phdr, const void* value, std::uint8_t value_size) {
    std::uint16_t found_offset = 0;
    std::uint16_t sentinel_offset = 0;
    if (scan(storage, phdr, found_offset, sentinel_offset)) {
        return storage.write_block(static_cast<std::uint16_t>(found_offset + sizeof(ParamHeader)), value, value_size);
    }
    if (found_offset == 0xFFFF) {
        return false; // scan ran off the end without ever finding a sentinel
    }

    const std::uint16_t ofs = found_offset; // == sentinel_offset: where the new entry goes
    if (static_cast<std::uint32_t>(ofs) + value_size + 2U * sizeof(ParamHeader) >= storage.size()) {
        return false; // not enough room for this entry plus a fresh sentinel after it
    }

    const ParamHeader new_sentinel = make_sentinel_header();
    const auto new_sentinel_ofs = static_cast<std::uint16_t>(ofs + sizeof(ParamHeader) + value_size);
    if (!storage.write_block(new_sentinel_ofs, &new_sentinel, sizeof(new_sentinel))) {
        return false;
    }
    if (!storage.write_block(static_cast<std::uint16_t>(ofs + sizeof(ParamHeader)), value, value_size)) {
        return false;
    }
    return storage.write_block(ofs, &phdr, sizeof(phdr)); // header written LAST - see file banner
}

} // namespace fwcpp::param
