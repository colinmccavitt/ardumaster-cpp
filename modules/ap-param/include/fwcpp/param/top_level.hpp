#pragma once

// Port of AP_Param::find() (AP_Param.cpp, ~line 919, read in full).
// CPP-043. See ADR-0013 for the AP_Param sub-effort's overall scoping
// and CPP-022's own notes for everything this builds on (find_group,
// get_base) - this is the piece CPP-022 stopped short of, deliberately,
// for lack of "a real vehicle-wide Info table this port doesn't have
// yet" (CPP-022 slice 3's own note). CPP-031 has since built Plane;
// this file is the generic (not Plane-specific) top-level search this
// port's own vehicle::Plane wiring (fwcpp/vehicle/plane.hpp's "CPP-043
// ADDENDUM") is the first real caller of.
//
// Mirrors upstream's real find() exactly, including both its branches:
//   - GROUP entries: case-SENSITIVE strncmp prefix match against
//     info.name (upstream: `strncmp(name, info.name, len) != 0`),
//     then dispatch into the already-ported find_group() (name_lookup.hpp)
//     for the rest of the name. Matches upstream's own "we continue
//     looking" comment: a non-matching or not-found group prefix falls
//     through to keep scanning later Info entries, rather than stopping
//     the whole search - this lets a top-level scalar share a name
//     prefix with a group (upstream's own example: "CAM_P_G").
//   - SCALAR entries: case-INsensitive strcasecmp direct name match
//     (upstream: `strcasecmp(name, info.name) == 0`) - a genuine
//     inconsistency with the GROUP branch's case-sensitive strncmp,
//     preserved exactly as CPP-022 slice 3's name_lookup.hpp banner
//     already documented for find_group's own analogous asymmetry (this
//     is upstream's own already-shipped behavior, not smoothed over per
//     ADR-0007 - "fix bugs" means bugs in the PORT, not upstream's).
//
// NOT reproduced: the `flags` out-parameter upstream's find() also
// fills in (via find_var_info, called on the found object to recover
// its own GroupInfo entry's flags) - find_var_info (by-pointer-identity
// self-discovery) is explicitly out of scope for CPP-043 (no real
// caller needs it yet - see plane.hpp's own "CPP-043 ADDENDUM"), so
// there is no way to recover a nested GROUP entry's flags here that
// isn't itself find_var_info. A future ticket that ports find_var_info
// can add flags back without redesigning this function's shape.
//
// kFlagInfoPointer (dynamically-defined top-level tables, used by
// AP_Scripting) is not reproduced here, matching find_group's own same
// exclusion - `info.group_info` is always read directly.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <strings.h> // POSIX strcasecmp - see name_lookup.hpp's own note (this port targets SITL/Linux)

#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/name_lookup.hpp>
#include <fwcpp/param/param.hpp> // VarType

namespace fwcpp::param {

// Searches `info_table` (terminated by a VarType::None sentinel entry,
// matching every other table in this module) for `name`. Returns
// nullptr if not found. On success, sets `ptype` and returns the
// resolved address as `void*` - see name_lookup.hpp's own banner for
// why not a typed/AP_Param*-shaped pointer.
[[nodiscard]] inline void* find(const char* name, const Info* info_table, VarType& ptype) {
    for (std::size_t i = 0; info_table[i].type != static_cast<std::uint8_t>(VarType::None); ++i) {
        const Info& info = info_table[i];
        const std::uint8_t type = info.type;
        if (type == static_cast<std::uint8_t>(VarType::Group)) {
            const std::size_t len = std::strlen(info.name);
            if (std::strncmp(name, info.name, len) != 0) {
                continue;
            }
            const GroupInfo* group_info = info.group_info;
            if (group_info == nullptr) {
                continue;
            }
            std::ptrdiff_t base = 0;
            if (!get_base(info, base)) {
                continue;
            }
            void* found = find_group(name + len, base, 0, group_info, ptype);
            if (found != nullptr) {
                return found;
            }
            // matches upstream: keep scanning later entries even after a
            // matching-prefix group came up empty.
        } else if (strcasecmp(name, info.name) == 0) {
            ptype = static_cast<VarType>(type);
            std::ptrdiff_t base = 0;
            if (!get_base(info, base)) {
                return nullptr;
            }
            return reinterpret_cast<void*>(base);
        }
    }
    return nullptr;
}

} // namespace fwcpp::param
